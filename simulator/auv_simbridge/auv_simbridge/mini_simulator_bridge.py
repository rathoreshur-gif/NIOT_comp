import cv2

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from auv_msgs.msg import GlobalForces, AuvState, StereoVisionFrame
from auv_msgs.msg import ThrusterForces, PsData

from std_msgs.msg import Bool, Float64, Int32
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Imu, Image
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import EntityWrench, Entity
from rosgraph_msgs.msg import Clock
from scipy.spatial.transform import Rotation as R
import numpy as np
from cv_bridge import CvBridge
from auv_simbridge.utils import transform_imu_from_gazebo
import subprocess
import signal
import yaml
import os
from ament_index_python.packages import get_package_share_directory


ROSBAG_TOPICS_CONFIG    = os.path.join(get_package_share_directory('auv_simbridge'), 'config', 'rosbag_topics.yaml')
SIMULATION_LOGS_DIR     = os.path.expanduser('~/simulation_logs')


def wrap(val):
    return (val + 180) % 360 - 180


class SimulatorBridge(Node):
    def __init__(self):
        super().__init__('simulator_node')

        self.clock_group = MutuallyExclusiveCallbackGroup()
        self.actuator_group = MutuallyExclusiveCallbackGroup()
        self.logging_group = MutuallyExclusiveCallbackGroup()
        self.front_camera_group = MutuallyExclusiveCallbackGroup()
        self.front_depth_group = MutuallyExclusiveCallbackGroup()
        self.bottom_camera_group = MutuallyExclusiveCallbackGroup()
        self.thrusters_killed = False
        self.front_image = None

        # Publishers
        self.localisation_pub = self.create_publisher(AuvState, '/localization/pose', 10)
        self.front_image_pub = self.create_publisher(Image, '/front_camera/image_raw', 1)
        self.front_image_depth_pub = self.create_publisher(StereoVisionFrame, '/front_camera/oakd_frame', 1)
        self.bottom_image_pub = self.create_publisher(Image, '/bottom_camera/image_raw', 1)
        self.payload_wrench_pub = self.create_publisher(EntityWrench, '/world/underwater_pool/wrench', 10)
        # Thruster publishers — 6-thruster config (4 surge + 2 heave)
        self.surge_front_left_pub  = self.create_publisher(Float64, '/model/auv/joint/surge_front_left_joint/cmd_thrust', 10)
        self.surge_front_right_pub = self.create_publisher(Float64, '/model/auv/joint/surge_front_right_joint/cmd_thrust', 10)
        self.surge_back_left_pub   = self.create_publisher(Float64, '/model/auv/joint/surge_back_left_joint/cmd_thrust', 10)
        self.surge_back_right_pub  = self.create_publisher(Float64, '/model/auv/joint/surge_back_right_joint/cmd_thrust', 10)
        self.heave_left_pub        = self.create_publisher(Float64, '/model/auv/joint/heave_left_joint/cmd_thrust', 10)
        self.heave_right_pub       = self.create_publisher(Float64, '/model/auv/joint/heave_right_joint/cmd_thrust', 10)
        self.ps_data_pub = self.create_publisher(PsData, '/ps/data', 10)
        # Republished IMU streams — consumed by auv_localization. Source is the
        # Gz-bridged /model/auv/imu_1, /model/auv/imu_2 (see imu1_callback).
        self.imu1_pub = self.create_publisher(Imu, '/imu/data_1', 50)
        self.imu2_pub = self.create_publisher(Imu, '/imu/data_2', 50)

        # Services
        self.kill_thrusters_srv = self.create_service(Empty, '/simulator/kill_thrusters', self.handle_kill_thrusters)
        self.unkill_thrusters_srv = self.create_service(Empty, '/simulator/unkill_thrusters', self.handle_unkill_thrusters)
        self.localisation_reset_service = self.create_service(Empty, '/localization/reset_service', self.localisation_reset_callback)
        self.start_logging_srv = self.create_service(Empty, '/simulator/start_logging', self.handle_start_logging, callback_group=self.logging_group)
        self.stop_logging_srv = self.create_service(Empty, '/simulator/stop_logging', self.handle_stop_logging, callback_group=self.logging_group)

        # Subscribers
        # All Gz→ROS topics are bridged in bridge_topics.yaml
        self.simulator_imu1_sub = self.create_subscription(Imu, '/model/auv/imu_1', self.imu1_callback, 10)
        self.simulator_imu2_sub = self.create_subscription(Imu, '/model/auv/imu_2', self.imu2_callback, 10)
        self.simulator_odometry_sub = self.create_subscription(Odometry, '/model/auv/odometry', self.odometry_callback, 10)
        self.simulator_depth_sub = self.create_subscription(Float64, '/model/auv/pressure', self.depth_callback, 10)
        self.global_forces_sub = self.create_subscription(GlobalForces, '/controller/global_forces', self.global_forces_callback, 10)
        self.thrust_force_sub = self.create_subscription(ThrusterForces, '/controller/thruster_forces', self.thruster_forces_callback, 10)
        self.keyboard_sub = self.create_subscription(Int32, '/keyboard/keypress', self.keyboard_callback, 10)
        self.simulator_pose_sub = self.create_subscription(PoseArray, '/model/auv/pose', self.pose_callback, 10)
        self.front_camera_sub = self.create_subscription(
            Image,
            '/model/auv/front_camera/image',
            self.front_camera_callback,
            1,
            callback_group=self.front_camera_group,
        )
        self.front_camera_depth_sub = self.create_subscription(
            Image,
            '/model/auv/front_camera/depth_image',
            self.front_camera_depth_callback,
            1,
            callback_group=self.front_depth_group,
        )
        self.bottom_camera_sub = self.create_subscription(
            Image,
            '/model/auv/bottom_camera/image',
            self.bottom_camera_callback,
            1,
            callback_group=self.bottom_camera_group,
        )
        self.sim_time_sub = self.create_subscription(Clock, '/clock', self.sim_time_callback, 10, callback_group=self.clock_group)

        self.bridge = CvBridge()
        self.pose_msg = AuvState()
        self.odom_msg = AuvState()
        self.current_time = 0.0
        self.previous_time = 0.0
        self.time_step = 0.0
        self.sec = 0
        self.nanosec = 0
        self.origin_state = AuvState()
        self.current_state = AuvState()
        self.origin_set = False
        # IMU Latched Reference State: the IMU orientation quaternion is
        # passed start-relative to match /localization/pose (which is
        # origin-subtracted). Latched on the first sample per IMU; reset
        # to None by `localisation_reset_callback` so the next sample
        # re-latches. Stored as scipy Rotation for clean composition.
        self._imu1_ref_quat = None
        self._imu2_ref_quat = None
        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.run)
        self.counter = 0
        self.rosbag_process = None
        self.gz_log_process = None
        with open(ROSBAG_TOPICS_CONFIG, 'r') as f:
            _config = yaml.safe_load(f)
        self.rosbag_topics = [
            t['name'] for t in _config['topics'] if not t.get('active', True)
        ]

    def quaternion_to_euler(self, x, y, z, w):
        r = R.from_quat([x, y, z, w])
        yaw, pitch, roll = r.as_euler('zyx', degrees=True)
        return roll, pitch, yaw

    def sim_time_callback(self, msg):
        self.sec = msg.clock.sec
        self.nanosec = msg.clock.nanosec
        self.current_time = self.sec + self.nanosec * 1e-9
        self.time_step = self.current_time - self.previous_time
        self.previous_time = self.current_time

    def front_camera_callback(self, msg):
        front_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self.front_image = front_image

        front_msg = self.bridge.cv2_to_imgmsg(front_image, encoding='rgb8')
        front_msg.header = msg.header
        self.front_image_pub.publish(front_msg)

    def bottom_camera_callback(self, msg):
        self.bottom_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        bottom_msg = self.bridge.cv2_to_imgmsg(self.bottom_image, encoding='rgb8')
        bottom_msg.header = msg.header
        self.bottom_image_pub.publish(bottom_msg)

    def front_camera_depth_callback(self, msg):
        front_image = self.front_image
        if front_image is None:
            return

        stereo_frame = StereoVisionFrame()
        stereo_frame.camera_frame = self.bridge.cv2_to_imgmsg(front_image, encoding='rgb8')
        depth_array = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_clipped = np.clip(depth_array, 0, 32767).astype(np.int16)
        stereo_frame.depth_frame = self.bridge.cv2_to_imgmsg(depth_clipped, encoding='16SC1')
        self.front_image_depth_pub.publish(stereo_frame)

    def run(self):
        self.counter += 1
        if self.counter % 1 == 0:
            self.current_state.header.stamp.sec = self.sec
            self.current_state.header.stamp.nanosec = self.nanosec
            self.current_state.position.x = (self.pose_msg.position.x - self.origin_state.position.x) * 100
            self.current_state.position.y = (self.pose_msg.position.y - self.origin_state.position.y) * 100
            self.current_state.position.z = (self.pose_msg.position.z - self.origin_state.position.z) * 100
            self.current_state.orientation.roll = wrap(self.pose_msg.orientation.roll - self.origin_state.orientation.roll)
            self.current_state.orientation.pitch = wrap(self.pose_msg.orientation.pitch - self.origin_state.orientation.pitch)
            self.current_state.orientation.yaw = wrap(self.pose_msg.orientation.yaw - self.origin_state.orientation.yaw)
            self.current_state.velocity.x = self.odom_msg.velocity.x * 100
            self.current_state.velocity.y = self.odom_msg.velocity.y * 100
            self.current_state.velocity.z = self.odom_msg.velocity.z * 100
            self.current_state.angular_velocity.x = self.odom_msg.angular_velocity.x
            self.current_state.angular_velocity.y = self.odom_msg.angular_velocity.y
            self.current_state.angular_velocity.z = self.odom_msg.angular_velocity.z
            self.localisation_pub.publish(self.current_state)
            depth_msg = PsData()
            # PsData.depth is metres below the START depth (positive down,
            # start-relative — matching the position.x/y subtraction above
            # and the localization smoother's z=0 seed). pose_msg.position.z
            # is already NED-down (positive below surface) from
            # pose_callback's negation of Gazebo Z, so the subtraction
            # alone gives the right sign — NO extra negation. Without
            # this subtraction the PS reads an absolute startup offset
            # (e.g. -45 cm with the prior sign flip) and pulls the
            # smoother's z away from its seed via the z_prior factor.
            depth_msg.depth = float(self.pose_msg.position.z
                                    - self.origin_state.position.z)
            self.ps_data_pub.publish(depth_msg)

    def pose_callback(self, msg):
        if msg.header.frame_id != 'odom':
            return
        self.pose_msg.position.x = msg.poses[0].position.x
        self.pose_msg.position.y = -msg.poses[0].position.y
        self.pose_msg.position.z = -msg.poses[0].position.z
        roll, pitch, yaw = self.quaternion_to_euler(
            msg.poses[0].orientation.x,
            msg.poses[0].orientation.y,
            msg.poses[0].orientation.z,
            msg.poses[0].orientation.w,
        )
        self.pose_msg.orientation.roll = wrap(roll)
        self.pose_msg.orientation.pitch = -wrap(pitch)
        self.pose_msg.orientation.yaw = -wrap(yaw)
        if not self.origin_set:
            self.localisation_reset_callback(None, None)
            self.origin_set = True

    def thruster_forces_callback(self, msg: ThrusterForces):
        if self.thrusters_killed:
            return
        surge_front_left  = Float64();
        surge_front_left.data  = msg.surge_front_left;
        self.surge_front_left_pub.publish(surge_front_left)
        surge_front_right = Float64();
        surge_front_right.data = msg.surge_front_right;
        self.surge_front_right_pub.publish(surge_front_right)
        surge_back_left   = Float64();
        surge_back_left.data   = msg.surge_back_left;
        self.surge_back_left_pub.publish(surge_back_left)
        surge_back_right  = Float64();
        surge_back_right.data  = msg.surge_back_right;
        self.surge_back_right_pub.publish(surge_back_right)
        heave_left  = Float64(); 
        heave_left.data  = -msg.heave_left; 
        self.heave_left_pub.publish(heave_left)
        heave_right = Float64(); 
        heave_right.data = -msg.heave_right; 
        self.heave_right_pub.publish(heave_right)
       
    def keyboard_callback(self, msg):
        if self.thrusters_killed:
            return

        key = msg.data
        # World-frame desired forces (forward/left/up, yaw CCW)
        world_forward, world_left, world_up, yaw_moment = 0.0, 0.0, 0.0, 0.0
        FORCE = 10.0

        if   key in (87, 16777235):  world_forward =  FORCE   # W / Up    — forward
        elif key in (83, 16777237):  world_forward = -FORCE   # S / Down  — backward
        elif key in (65, 16777234):  world_left    =  FORCE   # A / Left  — strafe left
        elif key in (68, 16777236):  world_left    = -FORCE   # D / Right — strafe right
        elif key == 81:              yaw_moment    =  FORCE   # Q         — yaw CCW
        elif key == 69:              yaw_moment    = -FORCE   # E         — yaw CW
        elif key == 82:              world_up      =  FORCE   # R         — heave up
        elif key == 70:              world_up      = -FORCE   # F         — heave down
        elif key == 75:                            # K         — hover
            self.surge_front_left_pub.publish(Float64(data=0.0))
            self.surge_front_right_pub.publish(Float64(data=0.0))
            self.surge_back_left_pub.publish(Float64(data=0.0))
            self.surge_back_right_pub.publish(Float64(data=0.0))
            self.heave_left_pub.publish(Float64(data=-2.924))
            self.heave_right_pub.publish(Float64(data=-2.924))
            return
        elif key == 32:                            # Space     — stop all
            for pub in (self.surge_front_left_pub, self.surge_front_right_pub,
                        self.surge_back_left_pub,  self.surge_back_right_pub,
                        self.heave_left_pub, self.heave_right_pub):
                pub.publish(Float64(data=0.0))
            return
        else:
            return

        # Rotate world-frame horizontal force into body frame using current yaw.
        # current_state.yaw is CW-positive, so negate to get math CCW convention.
        yaw_rad = np.deg2rad(-self.current_state.orientation.yaw)
        body_forward = world_forward * np.cos(yaw_rad) - world_left * np.sin(yaw_rad)
        body_left    = world_forward * np.sin(yaw_rad) + world_left * np.cos(yaw_rad)

        # X-thruster allocation: T = body_forward ± body_left ± yaw_moment
        surge_front_left  = body_forward - body_left - yaw_moment
        surge_front_right = body_forward + body_left + yaw_moment
        surge_back_left   = body_forward + body_left - yaw_moment
        surge_back_right  = body_forward - body_left + yaw_moment

        self.surge_front_left_pub.publish(Float64(data=surge_front_left))
        self.surge_front_right_pub.publish(Float64(data=surge_front_right))
        self.surge_back_left_pub.publish(Float64(data=surge_back_left))
        self.surge_back_right_pub.publish(Float64(data=surge_back_right))
        self.heave_left_pub.publish(Float64(data=world_up))
        self.heave_right_pub.publish(Float64(data=world_up))

    def handle_kill_thrusters(self, request, response):
        self.thrusters_killed = True
        self.get_logger().info('Thrusters killed.')
        return response

    def handle_unkill_thrusters(self, request, response):
        self.thrusters_killed = False
        self.get_logger().info('Thrusters unkilled.')
        return response


    def detach_payload(self, detach_pub):
        detach_pub.publish(Bool(data=True))

    def apply_payload_wrench(self, payload_name, force_xyz):
        wrench_msg = EntityWrench()
        wrench_msg.entity.name = f'auv::{payload_name}::{payload_name}_link'
        wrench_msg.entity.type = Entity.LINK
        wrench_msg.wrench.force.x = force_xyz[0]
        wrench_msg.wrench.force.y = force_xyz[1]
        wrench_msg.wrench.force.z = force_xyz[2]
        self.payload_wrench_pub.publish(wrench_msg)

    def forward_payload_force(self, force_n):
        yaw_rad = np.deg2rad(-self.current_state.orientation.yaw)
        world_forward = force_n * np.cos(yaw_rad)
        world_left = -force_n * np.sin(yaw_rad)
        return (world_forward, -world_left, 0.0)

    def handle_start_logging(self, request, response):
        import datetime
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        os.makedirs(SIMULATION_LOGS_DIR, exist_ok=True)

        if self.rosbag_process is None:
            bag_path = os.path.join(SIMULATION_LOGS_DIR, f'rosbag_{timestamp}')
            self.rosbag_process = subprocess.Popen(
                ['ros2', 'bag', 'record', '-o', bag_path] + self.rosbag_topics
            )
            self.get_logger().info(f'rosbag recording started: {bag_path}')
        else:
            self.get_logger().warn('rosbag already recording.')

        if self.gz_log_process is None:
            gz_path = os.path.join(SIMULATION_LOGS_DIR, f'gz_log_{timestamp}.tlog')
            self.gz_log_process = subprocess.Popen(['ign', 'log', 'record', '--file', gz_path])
            self.get_logger().info(f'gz log recording started: {gz_path}')
        else:
            self.get_logger().warn('gz log already recording.')

        return response

    def handle_stop_logging(self, request, response):
        import threading
        rosbag = self.rosbag_process
        gz_log = self.gz_log_process
        self.rosbag_process = None
        self.gz_log_process = None

        if rosbag is None and gz_log is None:
            self.get_logger().warn('No logging processes running.')
            return response

        # Signal subprocesses in a background thread so send_response() completes
        # before ros2 bag record begins DDS participant teardown (which can disrupt
        # the service response delivery if done synchronously in the callback).
        def _stop():
            import time
            time.sleep(0.2)
            for proc, name in [(rosbag, 'rosbag'), (gz_log, 'gz log')]:
                if proc is not None:
                    try:
                        proc.send_signal(signal.SIGINT)
                        self.get_logger().info(f'{name} recording stopping (finalizing in background).')
                    except Exception as e:
                        self.get_logger().warn(f'Failed to stop {name}: {e}')

        threading.Thread(target=_stop, daemon=True).start()
        return response

    def global_forces_callback(self, msg):
        pass

    def localisation_reset_callback(self, request, response):
        self.origin_state.position.x = self.pose_msg.position.x
        self.origin_state.position.y = self.pose_msg.position.y
        self.origin_state.position.z = self.pose_msg.position.z
        self.origin_state.orientation.roll = self.pose_msg.orientation.roll
        self.origin_state.orientation.pitch = self.pose_msg.orientation.pitch
        self.origin_state.orientation.yaw = self.pose_msg.orientation.yaw

        # Clear the IMU LRS so the next IMU sample re-latches the
        # reference attitude alongside the pose origin. Without this the
        # IMU orientation would stay relative to the OLD start while the
        # GT pose flips to the new start.
        self._imu1_ref_quat = None
        self._imu2_ref_quat = None

        self.get_logger().info('Localization origin reset.')
        return response

    def _convert_imu_to_ned(self, msg):
        # simbridge is the single ENU->NED boundary. Convert the raw Gz
        # IMU (ENU/FLU) into the project NED convention so EVERY topic on
        # the wire is NED. auv_localization undoes this internally (its
        # GTSAM smoother stays ENU); the round-trip is a no-op for the
        # estimate and exists purely for wire-convention uniformity.
        a, g, q = transform_imu_from_gazebo(
            (msg.linear_acceleration.x,
             msg.linear_acceleration.y,
             msg.linear_acceleration.z),
            (msg.angular_velocity.x,
             msg.angular_velocity.y,
             msg.angular_velocity.z),
            (msg.orientation.x, msg.orientation.y,
             msg.orientation.z, msg.orientation.w))
        (msg.linear_acceleration.x,
         msg.linear_acceleration.y,
         msg.linear_acceleration.z) = a
        (msg.angular_velocity.x,
         msg.angular_velocity.y,
         msg.angular_velocity.z) = g
        (msg.orientation.x, msg.orientation.y,
         msg.orientation.z, msg.orientation.w) = q

    def _apply_imu_lrs(self, msg, ref_attr):
        # Make IMU orientation start-relative so it matches the GT pose
        # convention on /localization/pose. Latch on first sample, then
        # publish q_ref.inv() * q_current using proper quaternion
        # composition (NOT Euler subtraction — which only happens to work
        # for near-level vehicles and breaks under combined roll/pitch).
        cur = R.from_quat([msg.orientation.x, msg.orientation.y,
                           msg.orientation.z, msg.orientation.w])
        ref = getattr(self, ref_attr)
        if ref is None:
            setattr(self, ref_attr, cur)
            # First sample IS the reference → relative is identity.
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0
            msg.orientation.w = 1.0
            return
        rel = ref.inv() * cur
        qx, qy, qz, qw = rel.as_quat()
        msg.orientation.x = float(qx)
        msg.orientation.y = float(qy)
        msg.orientation.z = float(qz)
        msg.orientation.w = float(qw)

    def imu1_callback(self, msg):
        self._convert_imu_to_ned(msg)
        self._apply_imu_lrs(msg, '_imu1_ref_quat')
        msg.header.frame_id = 'imu_1'
        self.imu1_pub.publish(msg)

    def imu2_callback(self, msg):
        self._convert_imu_to_ned(msg)
        self._apply_imu_lrs(msg, '_imu2_ref_quat')
        msg.header.frame_id = 'imu_2'
        self.imu2_pub.publish(msg)

    def odometry_callback(self, msg):
        roll = self.pose_msg.orientation.roll
        pitch = self.pose_msg.orientation.pitch
        yaw = self.pose_msg.orientation.yaw
        rotation = R.from_euler('zyx', [yaw, -pitch, -roll], degrees=True)
        velocities = rotation.apply([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        angular_velocities = rotation.apply([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
        self.odom_msg.velocity.x = velocities[0]
        self.odom_msg.velocity.y = -velocities[1]
        self.odom_msg.velocity.z = -velocities[2]
        self.odom_msg.angular_velocity.x = angular_velocities[0]
        self.odom_msg.angular_velocity.y = -angular_velocities[1]
        self.odom_msg.angular_velocity.z = -angular_velocities[2]

    def depth_callback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    simulator_node = SimulatorBridge()
    executor.add_node(simulator_node)
    executor.spin()
    simulator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()