"""
TeraSim Tick Driver

Continuously ticks the TeraSim simulation via HTTP API.
This node drives the simulation forward, publishes SUMO simulation
time to /clock, and signals other co-sim nodes via /terasim/tick_complete
so they sync exactly once per SUMO step (no timer-based polling).

Clock publishing is smoothed: each 0.1s SUMO step is published as
multiple small increments so Autoware's timer-driven nodes (which
use use_sim_time=true) see smooth time progression instead of jumps.
"""

import time
import requests
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64


class TeraSimTickDriver(Node):

    def __init__(self, **kwargs):
        super().__init__('terasim_tick_driver', **kwargs)

        # Declare parameters
        self.declare_parameter("http_host", "localhost")
        self.declare_parameter("http_port", 8000)
        self.declare_parameter("simulation_id", "")
        self.declare_parameter("clock_subdivisions", 10)

        self.http_host = self.get_parameter("http_host").value
        self.http_port = self.get_parameter("http_port").value
        self.simulation_id = self.get_parameter("simulation_id").value
        self.clock_subdivisions = self.get_parameter("clock_subdivisions").value

        self.base_url = f"http://{self.http_host}:{self.http_port}"
        self.session = requests.Session()

        # Clock publisher — publishes SUMO simulation time to /clock
        self.pub_clock = self.create_publisher(Clock, '/clock', 10)

        # Tick complete signal — other co-sim nodes subscribe to this
        # instead of using timers, so they sync exactly once per SUMO step
        self.pub_tick = self.create_publisher(Float64, '/terasim/tick_complete', 10)

        # Track previous sim time for smooth clock interpolation
        self._prev_sim_time = 0.0

        # Monotonic clock offset: when SUMO restarts (time jumps backward),
        # we add an offset so /clock never goes backward.  This prevents
        # Autoware's TF buffer from being corrupted by backward time jumps,
        # which causes the map to disappear in RViz.
        self._clock_offset = 0.0
        self._last_raw_sim_time = 0.0

        # Statistics
        self.tick_count = 0
        self.last_print_time = time.time()

        # Timer for ticking (as fast as possible, limited by simulation)
        self.timer = self.create_timer(0.001, self.on_timer)

        self.get_logger().info(
            f"Tick driver started: {self.base_url}, sim_id={self.simulation_id}, "
            f"clock_subdivisions={self.clock_subdivisions}"
        )

    def on_timer(self):
        """Send tick command and wait for completion."""
        if not self.simulation_id:
            return

        try:
            # Send tick
            self.session.post(
                f"{self.base_url}/simulation_tick/{self.simulation_id}",
                timeout=5
            )

            # Wait for tick to complete
            for _ in range(100):
                status = self.session.get(
                    f"{self.base_url}/simulation_status/{self.simulation_id}",
                    timeout=5
                ).json().get("status")

                if status in ("ticked", "finished"):
                    break
                time.sleep(0.01)

            if status == "finished":
                # Get simulation result with exit reason
                try:
                    result = self.session.get(
                        f"{self.base_url}/simulation_result/{self.simulation_id}",
                        timeout=5
                    ).json()
                    self.get_logger().info(f"Simulation finished: {result}")
                except Exception as e:
                    self.get_logger().info(f"Simulation finished (could not get result: {e})")
                # Cancel timer and raise KeyboardInterrupt to stop the executor
                self.timer.cancel()
                raise KeyboardInterrupt("Simulation finished")

            # Fetch simulation time and publish /clock smoothly + tick signal
            sim_time = self._get_sim_time()
            self._publish_clock_smooth(sim_time)
            self._publish_tick(sim_time)

            # Update statistics
            self.tick_count += 1
            now = time.time()
            if now - self.last_print_time >= 5.0:
                rate = self.tick_count / (now - self.last_print_time)
                self.get_logger().info(f"SUMO: {rate:.1f} ticks/sec")
                self.tick_count = 0
                self.last_print_time = now

        except Exception as e:
            self.get_logger().warn(f"Tick error: {e}")

    def _get_sim_time(self):
        """Fetch current simulation time from TeraSim."""
        try:
            resp = self.session.get(
                f"{self.base_url}/simulation/{self.simulation_id}/state",
                timeout=0.5
            )
            if resp.status_code == 200:
                return resp.json().get("simulation_time", 0.0)
        except Exception as e:
            self.get_logger().debug(f"Failed to get sim time: {e}")
        return 0.0

    def _make_monotonic(self, raw_sim_time):
        """Ensure clock never goes backward across SUMO restarts.

        When SUMO restarts, raw_sim_time resets to 0.  We detect this as a
        large backward jump and bump _clock_offset so the published clock
        continues forward from the last published value + one step.
        """
        if raw_sim_time < self._last_raw_sim_time - 0.5:
            # SUMO restarted — absorb the old time into the offset
            self._clock_offset += self._last_raw_sim_time + 0.1  # small gap
            self.get_logger().warn(
                f"SUMO time jumped backward ({self._last_raw_sim_time:.1f} -> "
                f"{raw_sim_time:.1f}), new clock_offset={self._clock_offset:.1f}"
            )
        self._last_raw_sim_time = raw_sim_time
        return raw_sim_time + self._clock_offset

    def _publish_clock_smooth(self, raw_sim_time):
        """Publish /clock in smooth increments to avoid timer burst issues.

        Instead of jumping from prev_time to sim_time in one message,
        publish N intermediate steps so Autoware nodes with use_sim_time=true
        see gradual time progression and their timers fire smoothly.
        """
        sim_time = self._make_monotonic(raw_sim_time)

        dt = sim_time - self._prev_sim_time
        if dt <= 0:
            # Time didn't advance, just publish current
            self._publish_clock(sim_time)
            self._prev_sim_time = sim_time
            return

        n = self.clock_subdivisions
        step = dt / n
        for i in range(1, n + 1):
            t = self._prev_sim_time + step * i
            self._publish_clock(t)
            if i < n:
                time.sleep(0.001)  # 1ms between sub-steps (~10ms total overhead)

        self._prev_sim_time = sim_time

    def _publish_clock(self, sim_time):
        """Publish a single /clock message."""
        clock_msg = Clock()
        clock_msg.clock.sec = int(sim_time)
        clock_msg.clock.nanosec = int((sim_time - int(sim_time)) * 1e9)
        self.pub_clock.publish(clock_msg)

    def _publish_tick(self, sim_time):
        """Signal co-sim nodes that a tick completed with current sim time."""
        msg = Float64()
        msg.data = sim_time
        self.pub_tick.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeraSimTickDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
