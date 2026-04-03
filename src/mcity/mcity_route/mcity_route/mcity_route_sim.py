#!/usr/bin/env python3

# Copyright 2022 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from tier4_system_msgs.srv import ChangeOperationMode, ChangeAutowareControl
from autoware_adapi_v1_msgs.srv import SetRoutePoints
from autoware_auto_system_msgs.msg import AutowareState


class McityRouteSim(Node):
    """
    Python implementation of the Autoware Interface Demo for Cosimulation.
    
    This node handles the interface between Autoware and the simulation environment,
    including localization initialization, route setting, and operation mode management.
    """

    def __init__(self):
        super().__init__('autoware_interface_demo_cosim')
        
        # Initialize state
        self.autoware_state = 1
        
        # Create publishers
        self.pub_local = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        
        # Create subscriptions
        self.sub_autoware_state = self.create_subscription(
            AutowareState,
            '/autoware/state',
            self.autoware_state_callback,
            10
        )
        
        # Create service clients
        self.cli_set_route_points = self.create_client(
            SetRoutePoints,
            '/planning/mission_planning/set_route_points'
        )
        
        self.cli_set_operation_mode = self.create_client(
            ChangeOperationMode,
            '/system/operation_mode/change_operation_mode'
        )
        
        self.cli_set_autoware_control = self.create_client(
            ChangeAutowareControl,
            '/system/operation_mode/change_autoware_control'
        )
        
        # Create timer (1000ms = 1 second)
        self.timer = self.create_timer(1.0, self.on_timer)
        
        self.get_logger().info("McityRouteSim initialized")

    def on_timer(self):
        """Timer callback that handles different Autoware states."""
        if self.autoware_state == AutowareState.INITIALIZING:
            self.init_localization()
            self.get_logger().info("Waiting for vehicle initialization...")
        elif self.autoware_state == AutowareState.WAITING_FOR_ROUTE:
            self.set_route_points()
            self.get_logger().info("Setting route points...")
        elif self.autoware_state == AutowareState.WAITING_FOR_ENGAGE:
            self.set_autoware_control(True)
            self.set_operation_mode(ChangeOperationMode.Request.AUTONOMOUS)
            self.get_logger().info("Enabling autoware control...")
        elif self.autoware_state == AutowareState.ARRIVED_GOAL:
            # Re-set the circular route so the AV loops indefinitely
            self.set_route_points()
            self.get_logger().info("Arrived at goal, re-setting circular route...")

    def init_localization(self):
        """Initialize localization with the exact same coordinates as the C++ version."""
        localization_msg = PoseWithCovarianceStamped()
        
        # Start of NCRC circular route (edge 44119073300)
        localization_msg.pose.pose.position.x = -2.47
        localization_msg.pose.pose.position.y = 1309.13
        localization_msg.pose.pose.position.z = 0.0

        localization_msg.pose.pose.orientation.x = 0.0
        localization_msg.pose.pose.orientation.y = 0.0
        localization_msg.pose.pose.orientation.z = 0.6975823055
        localization_msg.pose.pose.orientation.w = 0.7165046594
        
        # Set header
        localization_msg.header.stamp = self.get_clock().now().to_msg()
        localization_msg.header.frame_id = "map"
        
        # Publish
        self.pub_local.publish(localization_msg)

    def _make_pose(self, x, y, qz, qw):
        """Create a Pose with given position and orientation."""
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = 0.0
        p.orientation.x = 0.0
        p.orientation.y = 0.0
        p.orientation.z = qz
        p.orientation.w = qw
        return p

    def set_route_points(self):
        """Set route points tracing the full NCRC circular loop.

        Waypoints are sampled every ~10 edges along the 130-edge SUMO circular route,
        converted from SUMO coordinates to Autoware map frame using UTM_offset.
        The goal is set near the start so the route forms a complete circuit.
        """
        # Waypoints along the NCRC circular route (Autoware map frame)
        # Generated from SUMO edge midpoints with UTM_offset = (-4373.24, -4104.69)
        waypoints = [
            self._make_pose(-0.7700, 1372.6400, 0.6975823055, 0.7165046594),       # wp0: edge 44119073300
            self._make_pose(-914.4200, 1384.0700, -0.9507538173, 0.3099470583),     # wp1: edge 44114271901#0
            self._make_pose(-2237.6000, 983.9100, -0.9429141934, 0.3330357698),     # wp2: edge 4411427171#0
            self._make_pose(-3288.5200, 26.1400, -0.9736999601, 0.2278341234),      # wp3: edge 96869770
            self._make_pose(-3723.4100, -530.8200, -0.7268381348, 0.6868087986),    # wp4: edge 4414481741#0
            self._make_pose(-3760.5900, -1048.3600, -0.7332370654, 0.6799730920),   # wp5: edge 5139337630
            self._make_pose(-3913.6700, -1957.5700, -0.7372619639, 0.6756069838),   # wp6: edge 21387995030#0
            self._make_pose(-2903.0900, -2818.1700, -0.2788718999, 0.9603283102),   # wp7: edge 2577568300
            self._make_pose(-2201.3200, -2933.3300, 0.0029538355, 0.9999956374),    # wp8: edge 4421661840#0
            self._make_pose(-1063.2800, -3027.0500, -0.2939961851, 0.9558065930),   # wp9: edge 2135623740#0
            self._make_pose(510.8800, -3466.6500, -0.1535604377, 0.9881392574),     # wp10: edge 4421796671#0
            self._make_pose(369.8500, -1163.5700, 0.7042925037, 0.7099099022),      # wp11: edge 229035100
            self._make_pose(73.2900, -187.7300, 0.8364255164, 0.5480806104),        # wp12: edge 44160408600
        ]

        # Goal: near the start of the loop to complete the circuit
        goal = self._make_pose(-0.7700, 1372.6400, 0.6975823055, 0.7165046594)

        # Wait for service
        while not self.cli_set_route_points.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for the service. Exiting.")
                return
            self.get_logger().info("routing service not available, waiting again...")

        # Create request
        request = SetRoutePoints.Request()
        request.header.frame_id = "map"
        request.goal = goal
        request.waypoints = waypoints

        # Send request
        future = self.cli_set_route_points.call_async(request)
        self.get_logger().info("Setting NCRC circular route (13 waypoints)...")

    def set_operation_mode(self, mode):
        """Set operation mode."""
        request = ChangeOperationMode.Request()
        request.mode = mode

        while not self.cli_set_operation_mode.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for the service. Exiting.")
                return
            self.get_logger().info("operation mode service not available, waiting again...")

        future = self.cli_set_operation_mode.call_async(request)

    def set_autoware_control(self, autoware_control):
        """Set autoware control."""
        request = ChangeAutowareControl.Request()
        request.autoware_control = autoware_control

        while not self.cli_set_autoware_control.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for the service. Exiting.")
                return
            self.get_logger().info("autoware control service not available, waiting again...")

        future = self.cli_set_autoware_control.call_async(request)

    def autoware_state_callback(self, msg):
        """Callback for autoware state updates."""
        self.autoware_state = msg.state


def main(args=None):
    rclpy.init(args=args)
    
    node = McityRouteSim()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 