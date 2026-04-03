"""
NDE ↔ Autoware co-simulation bridge (ROS 2 node).

Responsibilities:
  1. Read NDE background vehicles from Redis key NDE_VEHICLES at 10 Hz and publish
     them as DetectedObjects on /perception/object_recognition/detection/objects.
  2. Subscribe to /localization/kinematic_state (Autoware AV odometry), convert
     coordinates to NDE local frame, and write to Redis key NDE_AV_STATE so the
     NDE sim can inject the AV into its simulation world.

Coordinate transform (computed once at startup):
  NDE local (x, y) uses origin from coordinate-transform.py:
      (42.229755621794645, -83.73991737197755) + manual alignment (+1.3, +2.0 m)
  Autoware map frame uses origin from aa_map/map_projector_info.yaml:
      (42.229392, -83.738943)
  Both origins are converted to UTM and the difference becomes a fixed 2D offset:
      autoware_x = nde_x + OFFSET_X
      nde_x      = autoware_x - OFFSET_X
"""

import json
import math
import redis
import rclpy
from math import atan2, cos, sin, pi
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from autoware_auto_perception_msgs.msg import (
    DetectedObjects,
    DetectedObject,
    ObjectClassification,
    Shape,
)


# ---------------------------------------------------------------------------
# Coordinate offset  (NDE local  →  Autoware map frame)
# ---------------------------------------------------------------------------

def _compute_coord_offset():
    """Return (offset_x, offset_y) to convert NDE local coords to Autoware map frame."""
    try:
        import utm
        nde_e, nde_n, _, _ = utm.from_latlon(42.229755621794645, -83.73991737197755)
        aw_e,  aw_n,  _, _ = utm.from_latlon(42.229392,           -83.738943)
        # Manual alignment corrections from coordinate-transform.py lines 9-10
        return nde_e - aw_e - 1.3, nde_n - aw_n - 2.0
    except ImportError:
        # utm not installed – fall back to approximate values computed offline
        # nde UTM ≈ (277415.6, 4686557.4), aw UTM ≈ (277497.1, 4686518.7)
        # offset_x = 277415.6 - 277497.1 - 1.3 = -82.8
        # offset_y = 4686557.4 - 4686518.7 - 2.0 = 36.7
        return -82.8, 36.7


OFFSET_X, OFFSET_Y = _compute_coord_offset()


# ---------------------------------------------------------------------------
# Helpers (mirrors autoware_vehicle_plugin.py:376-412)
# ---------------------------------------------------------------------------

def _heading_to_quaternion(heading_rad):
    """heading_rad: ENU yaw (East=0, CCW positive)  →  (qx, qy, qz, qw)"""
    return 0.0, 0.0, sin(heading_rad / 2.0), cos(heading_rad / 2.0)


def _quaternion_to_heading(qx, qy, qz, qw):
    """(qx, qy, qz, qw)  →  ENU yaw in radians"""
    heading = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    while heading >  pi: heading -= 2.0 * pi
    while heading < -pi: heading += 2.0 * pi
    return heading


def _autoware_to_center(x, y, heading_rad, rear_shaft_to_center=1.5):
    """Autoware reports rear-axle position; convert to vehicle centre."""
    return x + cos(heading_rad) * rear_shaft_to_center, y + sin(heading_rad) * rear_shaft_to_center


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------

class NDEBridge(Node):

    VEHICLES_KEY = "NDE_VEHICLES"
    AV_KEY       = "NDE_AV_STATE"

    def __init__(self):
        super().__init__("nde_bridge")

        self.redis = redis.Redis(host="localhost", port=6379, db=0)

        # Publish NDE background vehicles as DetectedObjects
        self.pub_objects = self.create_publisher(
            DetectedObjects,
            "/perception/object_recognition/detection/objects",
            10,
        )

        # Receive Autoware AV kinematic state
        self.sub_odom = self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self._odom_callback,
            10,
        )

        # 10 Hz publish timer
        self.create_timer(0.1, self._publish_timer)

        self.get_logger().info(
            f"NDE bridge started. Coord offset: OFFSET_X={OFFSET_X:.3f} m, OFFSET_Y={OFFSET_Y:.3f} m"
        )

    # ------------------------------------------------------------------
    # Timer: Redis NDE_VEHICLES → DetectedObjects
    # ------------------------------------------------------------------

    def _publish_timer(self):
        raw = self.redis.get(self.VEHICLES_KEY)
        if not raw:
            return

        try:
            vehicles = json.loads(raw)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse NDE_VEHICLES: {e}")
            return

        msg = DetectedObjects()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        for v in vehicles:
            obj = self._vehicle_dict_to_detected_object(v)
            msg.objects.append(obj)

        self.pub_objects.publish(msg)

    def _vehicle_dict_to_detected_object(self, v):
        obj = DetectedObject()
        obj.existence_probability = 1.0

        # Classification
        cls = ObjectClassification()
        cls.label = ObjectClassification.CAR
        cls.probability = 1.0
        obj.classification.append(cls)

        # Pose: convert NDE local → Autoware map frame
        aw_x = v["x"] + OFFSET_X
        aw_y = v["y"] + OFFSET_Y
        heading_rad = math.radians(v["heading_deg"])
        qx, qy, qz, qw = _heading_to_quaternion(heading_rad)

        pose_cov = PoseWithCovariance()
        pose_cov.pose.position.x = aw_x
        pose_cov.pose.position.y = aw_y
        pose_cov.pose.position.z = 0.8
        pose_cov.pose.orientation.x = qx
        pose_cov.pose.orientation.y = qy
        pose_cov.pose.orientation.z = qz
        pose_cov.pose.orientation.w = qw
        obj.kinematics.pose_with_covariance = pose_cov
        obj.kinematics.has_position_covariance = False
        obj.kinematics.orientation_availability = 0

        # Twist
        twist_cov = TwistWithCovariance()
        twist_cov.twist.linear.x = v.get("speed", 0.0)
        obj.kinematics.twist_with_covariance = twist_cov
        obj.kinematics.has_twist = True
        obj.kinematics.has_twist_covariance = False

        # Shape
        shape = Shape()
        shape.type = Shape.BOUNDING_BOX
        shape.dimensions.x = v.get("length", 3.6)
        shape.dimensions.y = v.get("width", 1.8)
        shape.dimensions.z = v.get("height", 1.5)
        obj.shape = shape

        return obj

    # ------------------------------------------------------------------
    # Subscription: Autoware /localization/kinematic_state → Redis NDE_AV_STATE
    # ------------------------------------------------------------------

    def _odom_callback(self, msg: Odometry):
        aw_x = msg.pose.pose.position.x
        aw_y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        heading_rad = _quaternion_to_heading(qx, qy, qz, qw)

        # Autoware reports rear-axle; convert to centre for NDE
        cx, cy = _autoware_to_center(aw_x, aw_y, heading_rad)

        # Autoware map frame → NDE local frame
        nde_x = cx - OFFSET_X
        nde_y = cy - OFFSET_Y

        speed = msg.twist.twist.linear.x  # longitudinal speed, m/s

        data = {
            "x": nde_x,
            "y": nde_y,
            "heading_deg": math.degrees(heading_rad),
            "speed": float(speed),
        }
        self.redis.set(self.AV_KEY, json.dumps(data))


def main(args=None):
    rclpy.init(args=args)
    node = NDEBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
