"""
Autoware Vehicle Plugin (HTTP)

Simple HTTP-based co-simulation plugin, based on the Redis version.
- When control_cav=True: Autoware controls CAV, send odom to TeraSim
- When control_cav=False: TeraSim controls CAV, publish TeraSim state to Autoware
- Always: Publish background vehicles to Autoware perception
"""

import math
import time
import rclpy
import tf2_ros
import numpy as np
import requests
from rclpy.node import Node
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose, PoseWithCovariance, TwistWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from autoware_auto_perception_msgs.msg import (
    DetectedObjects,
    DetectedObject,
    ObjectClassification,
    Shape,
    PredictedObjects,
    PredictedObject,
    PredictedObjectKinematics,
    PredictedPath,
)
from unique_identifier_msgs.msg import UUID as UUIDMsg
from builtin_interfaces.msg import Duration
import uuid
from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport
from std_msgs.msg import Float64

from math import atan2, cos, sin, pi


class AutowareVehiclePlugin(Node):

    def __init__(self, **kwargs):
        super().__init__("autoware_vehicle_plugin", **kwargs)

        # Declare parameters
        self.declare_parameter("http_host", "localhost")
        self.declare_parameter("http_port", 8000)
        self.declare_parameter("simulation_id", "")
        self.declare_parameter("control_cav", True)
        self.declare_parameter("perception_range", 150.0)  # meters, 0 = sync all
        # utm_offset = SUMO_net_utm_origin - Autoware_map_utm_origin
        # Always provided by TeraSim (scripts/autoware_cosim.py), auto-computed from
        # the SUMO .net.xml and Autoware map_projector_info.yaml. Do not set manually.
        self.declare_parameter("utm_offset_x", 0.0)
        self.declare_parameter("utm_offset_y", 0.0)

        self.http_host = self.get_parameter("http_host").value
        self.http_port = self.get_parameter("http_port").value
        self.simulation_id = self.get_parameter("simulation_id").value
        self.control_cav = self.get_parameter("control_cav").value
        self.perception_range = self.get_parameter("perception_range").value

        self.base_url = f"http://{self.http_host}:{self.http_port}"
        self.session = requests.Session()

        # Publishers - Autoware localization
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped,
            "/localization/pose_estimator/pose_with_covariance",
            10,
        )
        self.pub_twist = self.create_publisher(
            TwistWithCovarianceStamped,
            "/sensing/vehicle_velocity_converter/twist_with_covariance",
            10,
        )
        self.pub_odom = self.create_publisher(
            Odometry, "/localization/pose_twist_fusion_filter/kinematic_state", 10
        )

        # Publisher - Autoware perception
        self.pub_detected_objects = self.create_publisher(
            DetectedObjects, "/perception/object_recognition/detection/objects", 10
        )
        # Publisher - PredictedObjects for behavior_path_planner avoidance
        # behavior_path_planner subscribes to ~/input/perception which remaps to this topic
        self.pub_predicted_objects = self.create_publisher(
            PredictedObjects, "/perception/object_recognition/objects", 10
        )
        # Track object UUIDs for consistent identification across frames
        self._object_uuids = {}

        # Publishers - Autoware vehicle status (required for planning_simulator mode)
        self.pub_velocity_status = self.create_publisher(
            VelocityReport, "/vehicle/status/velocity_status", 10
        )
        self.pub_steering_status = self.create_publisher(
            SteeringReport, "/vehicle/status/steering_status", 10
        )

        # Subscriber - Autoware odometry
        # Must match publisher QoS: RELIABLE, KEEP_LAST(1), VOLATILE
        # Mismatched QoS (e.g., BEST_EFFORT subscriber) can cause message loss with CycloneDDS
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from rclpy.callback_groups import ReentrantCallbackGroup
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        # Use ReentrantCallbackGroup so subscription can run concurrently with timers
        # Without this, HTTP requests in timer callbacks can starve the subscription
        self._sub_callback_group = ReentrantCallbackGroup()
        self.sub_ego_odom = self.create_subscription(
            Odometry, "/localization/kinematic_state", self.odom_callback,
            odom_qos,
            callback_group=self._sub_callback_group
        )

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Event-driven sync: subscribe to tick_complete from tick driver
        # instead of polling on timers — sync exactly once per SUMO step
        self.sub_tick = self.create_subscription(
            Float64, '/terasim/tick_complete', self.on_tick, 10
        )

        # CAV timer runs on wall clock for responsive Autoware→SUMO updates
        # (Autoware odom arrives independently of SUMO ticks)
        self.cav_timer = self.create_timer(0.02, self.on_cav_timer)  # 50Hz CAV sync

        # State
        self.saved_odom_msg = Odometry()
        self.last_state = None
        self._log_count = 0
        self._last_sent_pos = (0.0, 0.0)  # Track last position sent to SUMO

        # Coordinate offset: Autoware local coords = SUMO local coords + UTM_offset
        # Formula: UTM_offset = sumo_net_offset - autoware_map_origin_UTM
        # For NCRC_Circular_Route (ncrc.net.xml):
        #   - SUMO net offset: (272722.10, 4681163.74)
        #   - Autoware origin UTM: (277095.34, 4685268.43)
        #   - UTM_offset = (272722.10 - 277095.34, 4681163.74 - 4685268.43) = (-4373.24, -4104.69)
        # For Ann Arbor full map (aa.net.xml):
        #   - SUMO net offset: (265795.09, 4673979.83)
        #   - UTM_offset = (265795.09 - 277095.34, 4673979.83 - 4685268.43) = (-11300.25, -11288.60)
        utm_x = self.get_parameter("utm_offset_x").value
        utm_y = self.get_parameter("utm_offset_y").value
        self.UTM_offset = [utm_x, utm_y, 0.0]

        self.get_logger().info(f"Vehicle plugin started: {self.base_url}, sim_id={self.simulation_id}")
        self.get_logger().info(f"control_cav={self.control_cav}, perception_range={self.perception_range}m, UTM_offset={self.UTM_offset}")

    def odom_callback(self, msg):
        """Store latest Autoware odometry."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f"[odom_callback] Received: x={x:.1f}, y={y:.1f}")
        self.saved_odom_msg = msg
        self._last_odom_time = self.get_clock().now()

    def _poll_odom_if_stale(self):
        """Fallback: if callback isn't working, read topic via subprocess."""
        # Check if we haven't received odom in a while
        if not hasattr(self, '_last_odom_time'):
            self._last_odom_time = self.get_clock().now()
            self._poll_counter = 0

        self._poll_counter = getattr(self, '_poll_counter', 0) + 1

        elapsed = (self.get_clock().now() - self._last_odom_time).nanoseconds / 1e9
        # Poll every 25 timer calls (~2Hz) if callback isn't working
        if elapsed > 0.5 and self._poll_counter % 25 == 0:
            self.get_logger().warn(f"[POLL] Callback stale for {elapsed:.1f}s, polling via subprocess...")
            try:
                import subprocess
                # Use ros2 topic echo to get the latest message (bypasses DDS subscription issues)
                result = subprocess.run(
                    ['ros2', 'topic', 'echo', '/localization/kinematic_state', '--once', '--no-arr'],
                    capture_output=True, text=True, timeout=1.0,  # Increased timeout
                    env={**__import__('os').environ, 'ROS_DOMAIN_ID': __import__('os').environ.get('ROS_DOMAIN_ID', '0')}
                )
                if result.returncode == 0 and result.stdout:
                    # Parse YAML output
                    import yaml
                    data = yaml.safe_load(result.stdout)
                    if data and 'pose' in data:
                        x = data['pose']['pose']['position']['x']
                        y = data['pose']['pose']['position']['y']
                        # Update saved_odom_msg position
                        self.saved_odom_msg.pose.pose.position.x = x
                        self.saved_odom_msg.pose.pose.position.y = y
                        if 'orientation' in data['pose']['pose']:
                            self.saved_odom_msg.pose.pose.orientation.x = data['pose']['pose']['orientation'].get('x', 0)
                            self.saved_odom_msg.pose.pose.orientation.y = data['pose']['pose']['orientation'].get('y', 0)
                            self.saved_odom_msg.pose.pose.orientation.z = data['pose']['pose']['orientation'].get('z', 0)
                            self.saved_odom_msg.pose.pose.orientation.w = data['pose']['pose']['orientation'].get('w', 1)
                        if 'twist' in data and 'twist' in data['twist']:
                            self.saved_odom_msg.twist.twist.linear.x = data['twist']['twist']['linear'].get('x', 0)
                        self._last_odom_time = self.get_clock().now()
                        self.get_logger().info(f"[SUBPROCESS_POLL] Updated odom: x={x:.1f}, y={y:.1f}")
                else:
                    self.get_logger().warn(f"[SUBPROCESS_POLL] Failed: returncode={result.returncode}, stderr={result.stderr[:100] if result.stderr else 'none'}")
            except subprocess.TimeoutExpired:
                self.get_logger().warn("[SUBPROCESS_POLL] Timeout - subprocess took too long")
            except Exception as e:
                self.get_logger().warn(f"[SUBPROCESS_POLL] Error: {e}")

    def _get_state(self):
        """Get simulation state from TeraSim HTTP API."""
        if not self.simulation_id:
            return None
        try:
            resp = self.session.get(
                f"{self.base_url}/simulation/{self.simulation_id}/state",
                timeout=0.1
            )
            if resp.status_code == 200:
                self.last_state = resp.json()
            return self.last_state
        except:
            return self.last_state

    def on_tick(self, msg):
        """Called once per SUMO step — sync BVs, TeraSim→Autoware CAV, and vehicle status."""
        # Sync BVs/VRUs to Autoware perception
        self.sync_terasim_vehicles_to_autoware()

        # If TeraSim controls CAV, push CAV state to Autoware and publish vehicle status
        # When control_cav=True, Autoware's simple_planning_simulator is the authoritative
        # source of vehicle status — publishing here would conflict and cause state machine issues
        if not self.control_cav:
            self.sync_terasim_cav_to_autoware()
            self._publish_vehicle_status()

    def on_cav_timer(self):
        """Sync Autoware CAV → TeraSim (wall clock timer for responsive updates)."""
        self._poll_odom_if_stale()
        if self.control_cav:
            self.sync_autoware_cav_to_terasim()

    def _publish_vehicle_status(self):
        """Publish vehicle status for Autoware planning_simulator mode."""
        stamp = self.get_clock().now().to_msg()

        # Velocity report
        vel_msg = VelocityReport()
        vel_msg.header.stamp = stamp
        vel_msg.header.frame_id = "base_link"
        vel_msg.longitudinal_velocity = self.saved_odom_msg.twist.twist.linear.x
        vel_msg.lateral_velocity = 0.0
        vel_msg.heading_rate = self.saved_odom_msg.twist.twist.angular.z
        self.pub_velocity_status.publish(vel_msg)

        # Steering report
        steer_msg = SteeringReport()
        steer_msg.stamp = stamp
        steer_msg.steering_tire_angle = 0.0
        self.pub_steering_status.publish(steer_msg)

    def sync_autoware_cav_to_terasim(self):
        """Send Autoware's CAV position to TeraSim (like Redis sync_autoware_cav_to_cosim)."""
        if not self.simulation_id:
            return

        # Get position from Autoware odometry
        aw_x = self.saved_odom_msg.pose.pose.position.x
        aw_y = self.saved_odom_msg.pose.pose.position.y

        # Skip if Autoware hasn't set a valid position yet (still at origin)
        if abs(aw_x) < 0.1 and abs(aw_y) < 0.1:
            return

        x = aw_x - self.UTM_offset[0]
        y = aw_y - self.UTM_offset[1]
        speed = self.saved_odom_msg.twist.twist.linear.x

        qx = self.saved_odom_msg.pose.pose.orientation.x
        qy = self.saved_odom_msg.pose.pose.orientation.y
        qz = self.saved_odom_msg.pose.pose.orientation.z
        qw = self.saved_odom_msg.pose.pose.orientation.w

        # Get orientation from quaternion
        orientation = self.get_orientation_from_quaternion(qx, qy, qz, qw)

        # Convert rear-axle to front bumper (Autoware uses rear-axle, SUMO uses front bumper)
        x, y = self.autoware_rear_axle_to_sumo(x, y, orientation)

        # Convert to SUMO angle (SUMO: 0=North, clockwise; ROS: 0=East, counter-clockwise)
        sumo_angle = (90 - math.degrees(orientation)) % 360

        # Log significant position changes (new 2D pose estimate)
        dx = x - self._last_sent_pos[0]
        dy = y - self._last_sent_pos[1]
        dist_change = math.sqrt(dx*dx + dy*dy)
        if dist_change > 10.0:  # More than 10m change
            self.get_logger().info(f"[AW->SUMO] Position jump detected! AW=({aw_x:.1f}, {aw_y:.1f}) -> SUMO=({x:.1f}, {y:.1f}), dist_change={dist_change:.1f}m")
        self._last_sent_pos = (x, y)

        # Debug logging every second
        self._log_count += 1
        if self._log_count % 50 == 0:
            self.get_logger().info(f"[AW->SUMO] AW=({aw_x:.1f}, {aw_y:.1f}) -> SUMO=({x:.1f}, {y:.1f}), angle={sumo_angle:.1f}°, speed={speed:.1f}")

        try:
            resp = self.session.post(
                f"{self.base_url}/simulation/{self.simulation_id}/agent_command",
                json={
                    "agent_id": "AV",
                    "agent_type": "vehicle",
                    "command_type": "set_state",
                    "data": {
                        "position": [x, y],
                        "sumo_angle": sumo_angle,
                        "speed": speed
                    }
                },
                timeout=0.1
            )
            if resp.status_code != 200:
                self.get_logger().warn(f"[AW->SUMO] HTTP error: {resp.status_code} - {resp.text}")
        except Exception as e:
            self.get_logger().debug(f"[AW->SUMO] Request failed: {e}")

    def sync_terasim_cav_to_autoware(self):
        """Publish TeraSim CAV position to Autoware (like Redis sync_cosim_cav_to_autoware)."""
        state = self._get_state()
        if not state:
            return

        vehicles = state.get("agent_details", {}).get("vehicle", {})
        cav = vehicles.get("CAV") or vehicles.get("AV")
        if not cav:
            return

        # Get position from TeraSim (SUMO local coords)
        cav_x = cav["x"] + self.UTM_offset[0]
        cav_y = cav["y"] + self.UTM_offset[1]
        cav_z = 0.0
        cav_orientation = cav.get("orientation", 0.0)
        cav_speed = cav.get("speed", 0.0)

        # Convert front bumper to rear-axle (SUMO uses front bumper, Autoware uses rear-axle)
        cav_x, cav_y = self.sumo_to_autoware_rear_axle(cav_x, cav_y, cav_orientation)

        # Get quaternion from orientation
        qx, qy, qz, qw = self.get_quaternion_from_orientation(cav_orientation)

        # Create and publish pose message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.pose.pose.position.x = cav_x
        pose_msg.pose.pose.position.y = cav_y
        pose_msg.pose.pose.position.z = cav_z
        pose_msg.pose.pose.orientation.x = qx
        pose_msg.pose.pose.orientation.y = qy
        pose_msg.pose.pose.orientation.z = qz
        pose_msg.pose.pose.orientation.w = qw
        pose_msg.pose.covariance = np.eye(6).flatten().tolist()

        # Create and publish twist message
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.twist.twist.linear.x = cav_speed
        twist_msg.twist.twist.linear.y = 0.0
        twist_msg.twist.twist.linear.z = 0.0
        twist_msg.twist.covariance = np.eye(6).flatten().tolist()

        # Create odom message
        odom_msg = Odometry()
        odom_msg.pose.pose = pose_msg.pose.pose
        odom_msg.twist.twist = twist_msg.twist.twist

        # Set headers
        stamp = self.get_clock().now().to_msg()
        header = Header()
        header.stamp = stamp
        header.frame_id = "map"
        pose_msg.header = header
        twist_msg.header = header
        odom_msg.header = header
        odom_msg.child_frame_id = "base_link"

        # Publish messages
        self.pub_pose.publish(pose_msg)
        self.pub_twist.publish(twist_msg)
        self.pub_odom.publish(odom_msg)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = cav_x
        t.transform.translation.y = cav_y
        t.transform.translation.z = cav_z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def sync_terasim_vehicles_to_autoware(self):
        """Publish background vehicles to Autoware perception (like Redis sync_cosim_vehicle_to_autoware)."""
        state = self._get_state()
        if not state:
            return

        stamp = self.get_clock().now().to_msg()

        detected_objects_msg = DetectedObjects()
        detected_objects_msg.header.stamp = stamp
        detected_objects_msg.header.frame_id = "map"

        # Also create PredictedObjects for behavior_path_planner avoidance
        predicted_objects_msg = PredictedObjects()
        predicted_objects_msg.header.stamp = stamp
        predicted_objects_msg.header.frame_id = "map"

        # BVs/VRUs are already filtered by perception_range on the TeraSim side
        vehicles = state.get("agent_details", {}).get("vehicle", {})

        # Process vehicles - both DetectedObjects and PredictedObjects
        # PredictedObjects needed for behavior_path_planner avoidance
        for veh_id, veh_info in vehicles.items():
            if veh_id in ("CAV", "AV"):
                continue
            detected_object = self._create_detected_object(veh_id, veh_info)
            detected_objects_msg.objects.append(detected_object)
            predicted_object = self._create_predicted_object(veh_id, veh_info)
            predicted_objects_msg.objects.append(predicted_object)

        # Process VRUs - both DetectedObjects and PredictedObjects
        vrus = state.get("agent_details", {}).get("vru", {})
        for vru_id, vru_info in vrus.items():
            detected_object = self._create_detected_object(vru_id, vru_info, is_pedestrian=True)
            detected_objects_msg.objects.append(detected_object)
            predicted_object = self._create_predicted_object(vru_id, vru_info, is_pedestrian=True)
            predicted_objects_msg.objects.append(predicted_object)

        # Process construction objects.
        #   WZ_*  — decorative cones spawned by UrbanConstructionZone. They exist
        #           only in the SUMO GUI / dash visualizer for visual flavor.
        #           Skip them here so Autoware perception (and the avoidance
        #           module) never sees them — only the real stalled vehicles
        #           (CONSTRUCTION_BV_*_stalled_object) drive avoidance behavior.
        #   CONSTRUCTION_BV_*  — real stalled vehicles, published normally.
        construction_objects = state.get("construction_objects", {})
        for obj_id, obj_info in construction_objects.items():
            if obj_id.startswith("WZ_"):
                continue
            detected_object = self._create_detected_object(obj_id, obj_info)
            detected_objects_msg.objects.append(detected_object)
            predicted_object = self._create_predicted_object(obj_id, obj_info)
            predicted_objects_msg.objects.append(predicted_object)

        self.pub_detected_objects.publish(detected_objects_msg)
        self.pub_predicted_objects.publish(predicted_objects_msg)

    def _create_detected_object(self, agent_id, agent_info, is_pedestrian=False):
        """Create DetectedObject message from agent info."""
        detected_object = DetectedObject()
        detected_object.existence_probability = 1.0

        # Classification - treat construction zone cones (WZ_* or CONSTRUCTION_*) as UNKNOWN obstacles
        classification = ObjectClassification()
        if is_pedestrian:
            classification.label = ObjectClassification.PEDESTRIAN
        elif agent_id.startswith("WZ_") or agent_id.startswith("CONSTRUCTION_"):
            # Construction zone cones - classify as UNKNOWN so planner treats as static obstacle
            classification.label = ObjectClassification.UNKNOWN
        else:
            classification.label = ObjectClassification.CAR
        classification.probability = 1.0
        detected_object.classification.append(classification)

        # Pose - convert SUMO front-bumper to vehicle center (DetectedObjects use center)
        x = agent_info["x"] + self.UTM_offset[0]
        y = agent_info["y"] + self.UTM_offset[1]
        orientation = agent_info.get("orientation", 0.0)
        vehicle_length = agent_info.get("length", 4.77)
        x, y = self.sumo_to_center(x, y, orientation, vehicle_length)

        pose = PoseWithCovariance()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.8
        pose.pose.orientation.w = cos(orientation / 2)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = sin(orientation / 2)
        detected_object.kinematics.pose_with_covariance = pose

        # Twist
        twist = TwistWithCovariance()
        twist.twist.linear.x = agent_info.get("speed", 0.0)
        detected_object.kinematics.twist_with_covariance = twist

        # Shape
        shape = Shape()
        shape.type = Shape.BOUNDING_BOX
        shape.dimensions.x = agent_info.get("length", 4.5)
        shape.dimensions.y = agent_info.get("width", 1.8)
        shape.dimensions.z = agent_info.get("height", 1.5)
        detected_object.shape = shape

        return detected_object

    def _create_predicted_object(self, agent_id, agent_info, is_pedestrian=False):
        """Create PredictedObject message with UUID for avoidance module.

        Uses autoware_auto_perception_msgs which has variable-length sequences
        (up to 100 elements) instead of fixed 100-element arrays.
        """
        predicted_object = PredictedObject()
        predicted_object.existence_probability = 1.0

        # Generate or retrieve consistent UUID for this object
        if agent_id not in self._object_uuids:
            self._object_uuids[agent_id] = uuid.uuid4()
        obj_uuid = self._object_uuids[agent_id]

        # Convert UUID to ROS message format (16 bytes)
        uuid_msg = UUIDMsg()
        uuid_msg.uuid = list(obj_uuid.bytes)
        predicted_object.object_id = uuid_msg

        # Classification - construction zone objects are UNKNOWN static obstacles
        classification = ObjectClassification()
        if is_pedestrian:
            classification.label = ObjectClassification.PEDESTRIAN
        elif agent_id.startswith("WZ_") or agent_id.startswith("CONSTRUCTION_"):
            classification.label = ObjectClassification.UNKNOWN
        else:
            classification.label = ObjectClassification.CAR
        classification.probability = 1.0
        predicted_object.classification.append(classification)

        # Kinematics - convert SUMO front-bumper to vehicle center
        x = agent_info["x"] + self.UTM_offset[0]
        y = agent_info["y"] + self.UTM_offset[1]
        orientation = agent_info.get("orientation", 0.0)
        vehicle_length = agent_info.get("length", 4.77)
        x, y = self.sumo_to_center(x, y, orientation, vehicle_length)
        speed = agent_info.get("speed", 0.0)
        qz = sin(orientation / 2)
        qw = cos(orientation / 2)

        kinematics = PredictedObjectKinematics()

        # Initial pose
        pose = PoseWithCovariance()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.8
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        kinematics.initial_pose_with_covariance = pose

        # Initial twist (static obstacle - zero velocity)
        twist = TwistWithCovariance()
        twist.twist.linear.x = speed
        kinematics.initial_twist_with_covariance = twist

        # Predicted path - need at least 2 poses for ObstacleCruisePlanner interpolation
        # (1 pose causes "Failed to find interpolated obstacle pose" and container crash)
        predicted_path = PredictedPath()
        predicted_path.confidence = 1.0
        predicted_path.time_step = Duration(sec=1, nanosec=0)  # 1 second time step

        # Current pose
        current_pose = Pose()
        current_pose.position.x = x
        current_pose.position.y = y
        current_pose.position.z = 0.8
        current_pose.orientation.x = 0.0
        current_pose.orientation.y = 0.0
        current_pose.orientation.z = qz
        current_pose.orientation.w = qw

        # Future pose (1 second ahead) - extrapolate from velocity
        future_pose = Pose()
        future_pose.position.x = x + speed * cos(orientation)
        future_pose.position.y = y + speed * sin(orientation)
        future_pose.position.z = 0.8
        future_pose.orientation.x = 0.0
        future_pose.orientation.y = 0.0
        future_pose.orientation.z = qz
        future_pose.orientation.w = qw
        predicted_path.path = [current_pose, future_pose]

        kinematics.predicted_paths = [predicted_path]
        predicted_object.kinematics = kinematics

        # Shape
        shape = Shape()
        shape.type = Shape.BOUNDING_BOX
        shape.dimensions.x = agent_info.get("length", 0.5)  # Cone default size
        shape.dimensions.y = agent_info.get("width", 0.5)
        shape.dimensions.z = agent_info.get("height", 1.0)
        predicted_object.shape = shape

        return predicted_object

    # Vehicle geometry (from vehicle_info.param.yaml):
    #   wheel_base=2.74, front_overhang=1.0, rear_overhang=1.03
    #   total_length = 1.0 + 2.74 + 1.03 = 4.77
    # Reference points:
    #   SUMO: front bumper
    #   Autoware base_link: rear axle
    #   Autoware DetectedObject: vehicle center
    REAR_AXLE_TO_FRONT = 2.74 + 1.0   # 3.74m (wheelbase + front overhang)
    FRONT_TO_CENTER = 4.77 / 2        # 2.385m (half vehicle length)

    def sumo_to_autoware_rear_axle(self, x, y, heading):
        """Convert SUMO front-bumper coordinate to Autoware rear-axle (base_link)."""
        x = x - math.cos(heading) * self.REAR_AXLE_TO_FRONT
        y = y - math.sin(heading) * self.REAR_AXLE_TO_FRONT
        return x, y

    def autoware_rear_axle_to_sumo(self, x, y, heading):
        """Convert Autoware rear-axle (base_link) to SUMO front-bumper coordinate."""
        x = x + math.cos(heading) * self.REAR_AXLE_TO_FRONT
        y = y + math.sin(heading) * self.REAR_AXLE_TO_FRONT
        return x, y

    def sumo_to_center(self, x, y, heading, vehicle_length=4.77):
        """Convert SUMO front-bumper coordinate to vehicle center (for DetectedObjects)."""
        x = x - math.cos(heading) * (vehicle_length / 2)
        y = y - math.sin(heading) * (vehicle_length / 2)
        return x, y

    def get_orientation_from_quaternion(self, qx, qy, qz, qw):
        """Extract yaw orientation from quaternion."""
        orientation = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        while orientation > pi:
            orientation -= 2.0 * pi
        while orientation < -pi:
            orientation += 2.0 * pi
        return orientation

    def get_quaternion_from_orientation(self, orientation):
        """Convert yaw orientation to quaternion."""
        w = cos(orientation / 2.0)
        x = 0.0
        y = 0.0
        z = sin(orientation / 2.0)
        return x, y, z, w


def main(args=None):
    rclpy.init(args=args)
    node = AutowareVehiclePlugin()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
