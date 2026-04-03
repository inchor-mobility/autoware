"""
Autoware Dummy Occupancy Grid

Publishes an empty occupancy grid centered on the CAV.
Required for Autoware's planning to work.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float64
from builtin_interfaces.msg import Time as TimeMsg


class AutowareDummyGrid(Node):

    def __init__(self, **kwargs):
        super().__init__('autoware_dummy_grid', **kwargs)

        self.saved_odom_msg = Odometry()
        self._sim_time = 0.0

        # Publisher
        self.pub_occ_grid = self.create_publisher(
            OccupancyGrid,
            '/perception/occupancy_grid_map/map',
            10
        )

        # Subscribers
        self.sub_ego_odom = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.odom_callback,
            10
        )
        self.sub_tick = self.create_subscription(
            Float64, '/terasim/tick_complete', self.on_tick, 10
        )

        # Initialize occupancy grid
        self.occ_grid = OccupancyGrid()
        self.occ_grid.info.resolution = 0.5
        self.occ_grid.info.origin.orientation.w = 1.0
        self.occ_grid.info.height = 300
        self.occ_grid.info.width = 300
        self.occ_grid.data = [0] * (300 * 300)

        self.get_logger().info("Dummy occupancy grid started")

    def odom_callback(self, msg):
        self.saved_odom_msg = msg

    def on_tick(self, msg):
        """Called once per SUMO step — publish occupancy grid."""
        self._sim_time = msg.data

        # Center grid on CAV position
        center_x = self.saved_odom_msg.pose.pose.position.x
        center_y = self.saved_odom_msg.pose.pose.position.y

        self.occ_grid.info.origin.position.x = center_x - 150
        self.occ_grid.info.origin.position.y = center_y - 150
        self.occ_grid.info.origin.position.z = 0.0

        self.occ_grid.header.frame_id = "map"
        self.occ_grid.header.stamp = self.get_clock().now().to_msg()

        self.pub_occ_grid.publish(self.occ_grid)


def main(args=None):
    rclpy.init(args=args)
    node = AutowareDummyGrid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
