import rclpy
from rclpy.node import MutuallyExclusiveCallbackGroup, Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from auv_msgs.msg import GlobalForces, AuvState, RGBDFrame, StereoVisionFrame
from auv_msgs.msg import ThrusterForces
from ros_gz_interfaces.msg import EntityWrench, Entity, Contacts
from std_msgs.msg import Bool, Float64, Int32, String
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Imu, Image
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from scipy.spatial.transform import Rotation as R
import numpy as np
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import subprocess
import signal
import yaml
import os
import threading
import cv2
from functools import partial

ROSBAG_TOPICS_CONFIG    = os.path.join(get_package_share_directory('auv_simbridge'), 'config', 'rosbag_topics.yaml')
SIMULATION_LOGS_DIR     = os.path.expanduser('~/simulation_logs')

# Per-finger closing angle. The joint limits in model.sdf allow up to 0.5 rad, but the
# fingers only need to reach the object far enough for both contact sensors to fire.
GRIPPER_CLOSE_ANGLE     = float(np.deg2rad(10.0))
# How long to give the fingers to retract before breaking the weld on release. See
# handle_open_gripper for why the two cannot happen at the same instant.
FINGER_RETRACT_TIME     = 0.6
# Both fingers must report the same object continuously for this long before the weld
# fires. The DetachableJoint welds at whatever pose it finds, so this second of the
# fingers squeezing at GRIPPER_CLOSE_ANGLE is what settles the prop into the jaw -
# weld on first touch instead and you capture whatever glancing pose that frame had.
GRASP_DWELL_TIME        = 1.0
# gz contact sensors publish only while a collision is live; they go silent on release
# rather than sending an empty Contacts. Nothing therefore tells us contact ended, so
# a finger counts as clear once its last message is this old. Without this the touch
# sets latch on forever and the first close_gripper welds the prop wherever it lies.
CONTACT_STALE_TIME      = 0.2
# Re-evaluates the dwell when no contact message is arriving, which is the only way
# the "contact stopped" edge is ever seen.
GRASP_POLL_PERIOD       = 0.05
# Models a finger can touch that are scenery rather than payload.
IGNORED_CONTACT_MODELS  = ('pool_floor', 'auv', 'octagon', 'task5_octagon')


def wrap(val):
    return (val + 180) % 360 - 180


class SimulatorBridge(Node):
    def __init__(self):
        super().__init__('simulator_bridge')
        self.thruster_counter = 0
        self.clock_group = MutuallyExclusiveCallbackGroup()
        self.actuator_group = MutuallyExclusiveCallbackGroup()
        self.logging_group = MutuallyExclusiveCallbackGroup()
        self.front_camera_group = MutuallyExclusiveCallbackGroup()
        self.front_depth_group = MutuallyExclusiveCallbackGroup()
        self.bottom_camera_group = MutuallyExclusiveCallbackGroup()
        self.thrusters_killed = False
        self.declare_parameter('legacy_keyboard', False)

        # Threading lock for shared image state
        self.front_image_lock = threading.Lock()
        self.front_image = None

        # Publishers
        self.localisation_pub = self.create_publisher(AuvState, '/localization/pose', 10)
        self.front_image_pub = self.create_publisher(Image, '/front_camera/image_raw', 10)
        self.front_image_depth_pub = self.create_publisher(StereoVisionFrame, '/front_camera/oakd_frame', 10)
        self.bottom_image_pub = self.create_publisher(Image, '/bottom_camera/image_raw', 10)
        self.payload_wrench_pub = self.create_publisher(EntityWrench, '/world/underwater_pool/wrench', 10)
        self.rgbd_pub = self.create_publisher(RGBDFrame, '/front_camera/rgbd_frame', 10)

        # Thruster publishers
        self.surge_front_left_pub  = self.create_publisher(Float64, '/model/auv/joint/surge_front_left_joint/cmd_thrust', 10)
        self.surge_front_right_pub = self.create_publisher(Float64, '/model/auv/joint/surge_front_right_joint/cmd_thrust', 10)
        self.surge_back_left_pub   = self.create_publisher(Float64, '/model/auv/joint/surge_back_left_joint/cmd_thrust', 10)
        self.surge_back_right_pub  = self.create_publisher(Float64, '/model/auv/joint/surge_back_right_joint/cmd_thrust', 10)
        self.heave_front_left_pub  = self.create_publisher(Float64, '/model/auv/joint/heave_front_left_joint/cmd_thrust', 10)
        self.heave_front_right_pub = self.create_publisher(Float64, '/model/auv/joint/heave_front_right_joint/cmd_thrust', 10)
        self.heave_back_left_pub   = self.create_publisher(Float64, '/model/auv/joint/heave_back_left_joint/cmd_thrust', 10)
        self.heave_back_right_pub  = self.create_publisher(Float64, '/model/auv/joint/heave_back_right_joint/cmd_thrust', 10)

        # Services
        self.kill_thrusters_srv = self.create_service(Empty, '/simulator/kill_thrusters', self.handle_kill_thrusters)
        self.unkill_thrusters_srv = self.create_service(Empty, '/simulator/unkill_thrusters', self.handle_unkill_thrusters)
        self.localisation_reset_service = self.create_service(Empty, '/localization/reset_service', self.localisation_reset_callback)
        self.start_logging_srv = self.create_service(Empty, '/simulator/start_logging', self.handle_start_logging, callback_group=self.logging_group)
        self.stop_logging_srv = self.create_service(Empty, '/simulator/stop_logging', self.handle_stop_logging, callback_group=self.logging_group)

        # Subscribers
        self.simulator_imu1_sub = self.create_subscription(Imu, '/model/auv/imu_1', self.imu1_callback, 10)
        self.simulator_imu2_sub = self.create_subscription(Imu, '/model/auv/imu_2', self.imu2_callback, 10)
        self.simulator_odometry_sub = self.create_subscription(Odometry, '/model/auv/odometry', self.odometry_callback, 10)
        self.simulator_depth_sub = self.create_subscription(Float64, '/model/auv/pressure', self.depth_callback, 10)
        self.global_forces_sub = self.create_subscription(GlobalForces, '/controller/global_forces', self.global_forces_callback, 10)
        self.thrust_force_sub = self.create_subscription(ThrusterForces, '/controller/thruster_forces', self.thruster_forces_callback, 10)
        self.keyboard_sub = self.create_subscription(Int32, '/keyboard/keypress', self.keyboard_callback, 10)
        self.simulator_pose_sub = self.create_subscription(PoseArray, '/model/auv/pose', self.pose_callback, 10)
        
        # Camera subscriptions using qos_profile_sensor_data to prevent network bottlenecks
        self.front_camera_sub = self.create_subscription(
            Image,
            '/model/auv/front_camera/image',
            self.front_camera_callback,
            qos_profile_sensor_data,
            callback_group=self.front_camera_group,
        )
        self.front_camera_depth_sub = self.create_subscription(
            Image,
            '/model/auv/front_camera/depth_image',
            self.front_camera_depth_callback,
            qos_profile_sensor_data,
            callback_group=self.front_depth_group,
        )
        self.bottom_camera_sub = self.create_subscription(
            Image,
            '/model/auv/bottom_camera/image',
            self.bottom_camera_callback,
            qos_profile_sensor_data,
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

        self._init_distortion_maps()


        #Gripper start
        self.left_touching = set()
        self.right_touching = set()
        # Sim time each finger last reported a contact. Paired with CONTACT_STALE_TIME
        # to age the sets out, since Gazebo never says "contact ended".
        self.left_contact_stamp = 0.0
        self.right_contact_stamp = 0.0
        # The object both fingers currently agree on, and the sim time they started
        # agreeing. Cleared the moment either finger loses it, so the dwell only ever
        # counts one unbroken stretch of contact.
        self.grasp_candidate = None
        self.grasp_since = None
        self.gripper_group = MutuallyExclusiveCallbackGroup()

        # Publisher for the rest of the ROS 2 stack to know what is held
        self.grasped_object_pub = self.create_publisher(String, '/model/auv/grasped_object', 10)

        # Contact Subscribers
        self.left_contact_sub = self.create_subscription(
            Contacts,
            '/model/auv/left_finger_contact',
            self.left_contact_callback,
            qos_profile_sensor_data,
            callback_group=self.gripper_group,
        )
        self.right_contact_sub = self.create_subscription(
            Contacts,
            '/model/auv/right_finger_contact',
            self.right_contact_callback,
            qos_profile_sensor_data,
            callback_group=self.gripper_group,
        )
        
        # Gripper position publishers
        self.left_finger_pub = self.create_publisher(
            Float64, '/model/m7urdfnew_sdf_package/joint/left_finger_joint/cmd_pos', 10) 
        self.right_finger_pub = self.create_publisher(
            Float64, '/model/m7urdfnew_sdf_package/joint/right_finger_joint/cmd_pos', 10) 

        # Marker droppers. The DetachableJoint plugins for both markers live in
        # the AUV model and auto-attach at world load, so a marker rides along
        # until its detach topic is poked.
        self.marker_drop_pubs = {
            'left': self.create_publisher(EmptyMsg, '/model/auv/marker_left/drop', 10),
            'right': self.create_publisher(EmptyMsg, '/model/auv/marker_right/drop', 10),
        }
        self.markers_dropped = set()
        self.drop_left_srv = self.create_service(
            Empty, '/simulator/drop_marker_left',
            lambda req, res: self.handle_drop_marker('left', res))
        self.drop_right_srv = self.create_service(
            Empty, '/simulator/drop_marker_right',
            lambda req, res: self.handle_drop_marker('right', res))

        # Gripper actuation services
        self.close_gripper_srv = self.create_service(
            Empty, '/simulator/close_gripper', self.handle_close_gripper)
        self.open_gripper_srv = self.create_service(
            Empty, '/simulator/open_gripper', self.handle_open_gripper)

        # Objects that have a DetachableJoint declared against base_link in model.sdf.
        # Anything not listed here is reported as grasped but cannot be welded on.
        # One entry per plugin instance; the name is the model name Gazebo reports in
        # the contact message AND the topic namespace bridge_topics_thrusters.yaml
        # publishes under, so all three have to stay spelled the same.
        # Not every world carries every one of these - test_matsya.sdf has only
        # test_box, the competition course only the four pickups - and a joint whose
        # child model is missing simply never reports, which is handled below.
        self.attachable_objects = (
            'test_box',
            'pickup_bandaid',
            'pickup_plug',
            'pickup_capsule',
            'pickup_screw',
        )
        self.attached_object = None
        self.initial_release_done = False
        self.release_attempts = 0
        # Attaching is gated on an explicit close_gripper call. Brushing past an object
        # with the fingers open must never pick it up.
        self.gripper_commanded_closed = False
        # Contact callbacks live in gripper_group, the services in the default group, so
        # the MultiThreadedExecutor can run them at once. Reentrant because the close
        # service takes the lock and then calls into check_grasp_condition.
        self.gripper_lock = threading.RLock()

        self.gripper_attach_pubs = {}
        self.gripper_detach_pubs = {}
        self.gripper_joint_state_subs = {}
        # Joints Gazebo has confirmed open. Only the ones that ever report can end up
        # here, so the startup release below waits on its attempt cap rather than on
        # this filling up - a world with two of the five would otherwise wait forever.
        self.joints_released = set()
        for obj in self.attachable_objects:
            self.gripper_attach_pubs[obj] = self.create_publisher(
                EmptyMsg, f'/model/auv/gripper/{obj}/attach', 10)
            self.gripper_detach_pubs[obj] = self.create_publisher(
                EmptyMsg, f'/model/auv/gripper/{obj}/detach', 10)
            self.gripper_joint_state_subs[obj] = self.create_subscription(
                String,
                f'/model/auv/gripper/{obj}/joint_state',
                partial(self.gripper_joint_state_callback, obj),
                10,
                callback_group=self.gripper_group,
            )
        # gz-sim 8's DetachableJoint welds itself at world load, so release it as early
        # as we can. Cancels itself once every joint that exists has confirmed open.
        self.initial_release_timer = self.create_timer(
            0.1, self.release_initial_attachment, callback_group=self.gripper_group)

        # Fires once, FINGER_RETRACT_TIME after an open request, to break the weld only
        # after the fingers have physically cleared the object. Parked until needed.
        self.deferred_release_timer = self.create_timer(
            FINGER_RETRACT_TIME,
            self.deferred_release,
            callback_group=self.gripper_group,
            autostart=False,
        )

        # Contact callbacks only fire while something is touching, so on their own they
        # can never observe the moment contact stops. This ticks the same evaluation so
        # the dwell gets reset when the fingers come away empty.
        self.grasp_poll_timer = self.create_timer(
            GRASP_POLL_PERIOD,
            self.check_grasp_condition,
            callback_group=self.gripper_group,
        )
        #gripper end

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
    
    def _init_distortion_maps(self):
        # sim_w/sim_h and f_sim describe the SOURCE image coming out of Gazebo and
        # must track the <image> size in m7urdfnew_sdf_package/model.sdf. K and
        # out_w/out_h describe the rectified image we publish, which is unchanged.
        def create_map(K, D, f_sim, out_w=1280, out_h=720, sim_w=700, sim_h=400):
            U, V = np.meshgrid(np.arange(out_w), np.arange(out_h))
            pts = np.stack([U, V], axis=-1).astype(np.float32).reshape(-1, 1, 2)
            
            ideal_norm = cv2.undistortPoints(pts, K, D).reshape(-1, 2)
            
            map_x = (ideal_norm[:, 0] * f_sim + (sim_w / 2.0)).reshape(out_h, out_w).astype(np.float32)
            map_y = (ideal_norm[:, 1] * f_sim + (sim_h / 2.0)).reshape(out_h, out_w).astype(np.float32)
            
            return map_x, map_y

        # FRONT CAMERA (Water-Adjusted)
        # Old fx: 571.07 * 1.333 = 761.24
        # Old fy: 570.58 * 1.333 = 760.58
        front_K = np.array([[761.24, 0, 609.27], [0, 760.58, 352], [0, 0, 1]], dtype=np.float32)
        # Clean 5-parameter model simulating flat-port pincushion distortion
        front_D = np.array([0.15, -0.02, 0.0, 0.0, 0.0], dtype=np.float32)
        # 380.62 = 761.24 / 2, matching the halved sim image width.
        self.front_map_x, self.front_map_y = create_map(front_K, front_D, 380.62)

        # BOTTOM CAMERA (Water-Adjusted)
        # Old fx: 564.58 * 1.333 = 752.59
        # Old fy: 564.82 * 1.333 = 752.91
        bottom_K = np.array([[752.59, 0, 643.28], [0, 752.91, 382.55], [0, 0, 1]], dtype=np.float32)
        bottom_D = np.array([0.15, -0.02, 0.0, 0.0, 0.0], dtype=np.float32)
        # 376.30 = 752.59 / 2, matching the halved sim image width.
        self.bottom_map_x, self.bottom_map_y = create_map(bottom_K, bottom_D, 376.30)
    
    def front_camera_depth_callback(self, msg):
        # Thread-safe read and copy (This is already the distorted RGB image from your updated callback)
        with self.front_image_lock:
            if self.front_image is None:
                return
            current_front_image = self.front_image.copy()

        self.stereo_frame = StereoVisionFrame()
        self.stereo_frame.camera_frame = self.bridge.cv2_to_imgmsg(current_front_image, encoding='rgb8')

        # 1. Pull raw simulation depth array from Gazebo
        depth_array = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # 2. Apply the exact same distortion map using nearest-neighbor and BORDER_REPLICATE
        distorted_depth = cv2.remap(
            depth_array,
            self.front_map_x,
            self.front_map_y,
            interpolation=cv2.INTER_NEAREST
        )

        # 3. Process the distorted depth map into your standard message format
        depth_mm = distorted_depth.astype(np.float32) * 1000.0
        depth_clipped = np.clip(depth_mm, 0, 32767).astype(np.int16)
        self.stereo_frame.depth_frame = depth_clipped.flatten().tolist()

        self.front_image_depth_pub.publish(self.stereo_frame)

        # --- NEW: combined RGBDFrame publishing ---
        rgbd_frame = RGBDFrame()
        rgbd_frame.header = msg.header

        rgbd_frame.rgb = self.bridge.cv2_to_imgmsg(current_front_image, encoding='rgb8')
        rgbd_frame.rgb.header = msg.header

        # Depth as float32 meters in standard '32FC1' image encoding
        rgbd_frame.depth = self.bridge.cv2_to_imgmsg(
            distorted_depth.astype(np.float32), encoding='32FC1'
        )
        rgbd_frame.depth.header = msg.header

        self.rgbd_pub.publish(rgbd_frame)

        
    def front_camera_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        
        # Apply Artificial Lens Distortion and fill the mathematical voids
        distorted_image = cv2.remap(
            cv_image, 
            self.front_map_x, 
            self.front_map_y, 
            interpolation=cv2.INTER_LINEAR
        )
        
        # Thread-safe write
        with self.front_image_lock:
            self.front_image = distorted_image
            
        self.front_image_pub.publish(self.bridge.cv2_to_imgmsg(distorted_image, encoding='rgb8'))

    def bottom_camera_callback(self, msg):
        # 1. Pull into OpenCV 
        self.bottom_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

        # 2. Apply Artificial Lens Distortion and fill the mathematical voids
        distorted_bottom = cv2.remap(
            self.bottom_image, 
            self.bottom_map_x, 
            self.bottom_map_y, 
            interpolation=cv2.INTER_LINEAR
        )

        # 3. Apply the tuned wash-out effect
        final_image = cv2.convertScaleAbs(distorted_bottom, alpha=0.25, beta=73)

        # 4. Publish back to ROS 2
        self.bottom_image_pub.publish(self.bridge.cv2_to_imgmsg(final_image, encoding='rgb8'))

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

    def pose_callback(self, msg):
        pass

    def thruster_forces_callback(self, msg: ThrusterForces):
        # self.thruster_counter += 1
        # if self.thruster_counter%100 == 0:
            if self.thrusters_killed:
                return
            surge_front_left  = Float64(); surge_front_left.data  = msg.surge_front_left;  self.surge_front_left_pub.publish(surge_front_left)
            surge_front_right = Float64(); surge_front_right.data = msg.surge_front_right; self.surge_front_right_pub.publish(surge_front_right)
            surge_back_left   = Float64(); surge_back_left.data   = msg.surge_back_left;   self.surge_back_left_pub.publish(surge_back_left)
            surge_back_right  = Float64(); surge_back_right.data  = msg.surge_back_right;  self.surge_back_right_pub.publish(surge_back_right)
            heave_front_left  = Float64(); heave_front_left.data  = -msg.heave_front_left;  self.heave_front_left_pub.publish(heave_front_left)
            heave_front_right = Float64(); heave_front_right.data = -msg.heave_front_right; self.heave_front_right_pub.publish(heave_front_right)
            heave_back_left   = Float64(); heave_back_left.data   = -msg.heave_back_left;   self.heave_back_left_pub.publish(heave_back_left)
            heave_back_right  = Float64(); heave_back_right.data  = -msg.heave_back_right;  self.heave_back_right_pub.publish(heave_back_right)

    def keyboard_callback(self, msg):
        # Superseded by auv_simbridge/teleop.py, which flies the vehicle closed
        # loop instead of publishing raw thrust. Both would fight over the same
        # thruster topics, so this stays off unless explicitly re-enabled with
        #     ros2 param set /simulator_bridge legacy_keyboard true
        if not self.get_parameter('legacy_keyboard').value:
            return
        if self.thrusters_killed:
            return

        key = msg.data
        world_forward, world_left, world_up, yaw_moment = 0.0, 0.0, 0.0, 0.0
        FORCE = 10.0

        if   key in (87, 16777235):  world_forward =  FORCE
        elif key in (83, 16777237):  world_forward = -FORCE
        elif key in (65, 16777234):  world_left    =  FORCE
        elif key in (68, 16777236):  world_left    = -FORCE
        elif key == 81:              yaw_moment    =  FORCE
        elif key == 69:              yaw_moment    = -FORCE
        elif key == 82:              world_up      =  FORCE
        elif key == 70:              world_up      = -FORCE
        elif key == 75:
            self.surge_front_left_pub.publish(Float64(data=0.0))
            self.surge_front_right_pub.publish(Float64(data=0.0))
            self.surge_back_left_pub.publish(Float64(data=0.0))
            self.surge_back_right_pub.publish(Float64(data=0.0))
            self.heave_front_left_pub.publish(Float64(data=-1.462))
            self.heave_front_right_pub.publish(Float64(data=-1.462))
            self.heave_back_left_pub.publish(Float64(data=-1.462))
            self.heave_back_right_pub.publish(Float64(data=-1.462))
            return
        elif key == 32:
            for pub in (self.surge_front_left_pub, self.surge_front_right_pub,
                        self.surge_back_left_pub,  self.surge_back_right_pub,
                        self.heave_front_left_pub, self.heave_front_right_pub,
                        self.heave_back_left_pub,  self.heave_back_right_pub):
                pub.publish(Float64(data=0.0))
            return
        else:
            return

        yaw_rad = np.deg2rad(-self.current_state.orientation.yaw)
        body_forward = world_forward * np.cos(yaw_rad) - world_left * np.sin(yaw_rad)
        body_left    = world_forward * np.sin(yaw_rad) + world_left * np.cos(yaw_rad)

        surge_front_left  = body_forward - body_left - yaw_moment
        surge_front_right = body_forward + body_left + yaw_moment
        surge_back_left   = body_forward + body_left - yaw_moment
        surge_back_right  = body_forward - body_left + yaw_moment

        self.surge_front_left_pub.publish(Float64(data=surge_front_left))
        self.surge_front_right_pub.publish(Float64(data=surge_front_right))
        self.surge_back_left_pub.publish(Float64(data=surge_back_left))
        self.surge_back_right_pub.publish(Float64(data=surge_back_right))
        self.heave_front_left_pub.publish(Float64(data=world_up))
        self.heave_front_right_pub.publish(Float64(data=world_up))
        self.heave_back_left_pub.publish(Float64(data=world_up))
        self.heave_back_right_pub.publish(Float64(data=world_up))

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

        self.get_logger().info('Localization origin reset.')
        return response

    def imu1_callback(self, msg):
        pass

    def imu2_callback(self, msg):
        pass

    def odometry_callback(self, msg):
        pose = msg.pose.pose
        twist = msg.twist.twist

        self.pose_msg.position.x = pose.position.x
        self.pose_msg.position.y = -pose.position.y
        self.pose_msg.position.z = -pose.position.z

        roll, pitch, yaw = self.quaternion_to_euler(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        self.pose_msg.orientation.roll = wrap(roll)
        self.pose_msg.orientation.pitch = -wrap(pitch)
        self.pose_msg.orientation.yaw = -wrap(yaw)

        rotation = R.from_quat([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ])
        velocities = rotation.apply([
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
        ])
        angular_velocities = [
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        ]

        self.odom_msg.velocity.x = velocities[0]
        self.odom_msg.velocity.y = -velocities[1]
        self.odom_msg.velocity.z = -velocities[2]
        self.odom_msg.angular_velocity.x = angular_velocities[0]
        self.odom_msg.angular_velocity.y = -angular_velocities[1]
        self.odom_msg.angular_velocity.z = -angular_velocities[2]

        if not self.origin_set:
            self.get_logger().info("LRS in simulator")
            self.localisation_reset_callback(None, None)
            self.origin_set = True

    def depth_callback(self, msg):
        pass

    def left_contact_callback(self, msg):
        # Update left finger state
        with self.gripper_lock:
            self.left_touching = self.extract_models(msg, "left_finger_collision")
            self.left_contact_stamp = self.current_time
        self.check_grasp_condition()

    def right_contact_callback(self, msg):
        # Update right finger state
        with self.gripper_lock:
            self.right_touching = self.extract_models(msg, "right_finger_collision")
            self.right_contact_stamp = self.current_time
        self.check_grasp_condition()

    def extract_models(self, msg, finger_collision_name):
        """ Parses the Gazebo contact message to extract the model name of the target. """
        touched_models = set()
        
        # msg.contact is an array of Contact messages
        for contact_data in msg.contacts:
            col1 = contact_data.collision1.name
            col2 = contact_data.collision2.name

            # Gazebo formats contact names as 'model_name::link_name::collision_name'
            if finger_collision_name not in col1:
                model_name = col1.split("::")[0]
                touched_models.add(model_name)
            elif finger_collision_name not in col2:
                model_name = col2.split("::")[0]
                touched_models.add(model_name)
                
        return touched_models

    def live_touching(self, now):
        """
        The two touch sets with stale ones treated as empty.

        Gazebo's contact sensor publishes only while a collision is live and sends
        nothing at all on release, so self.left_touching / self.right_touching keep
        their last value indefinitely. Anything older than CONTACT_STALE_TIME is a
        finger that has let go, not a finger that is still holding on.
        """
        left = self.left_touching if (now - self.left_contact_stamp) < CONTACT_STALE_TIME else set()
        right = self.right_touching if (now - self.right_contact_stamp) < CONTACT_STALE_TIME else set()
        return left, right

    def reset_grasp_tracking(self):
        """ Forgets all contact state. Caller must hold gripper_lock. """
        self.left_touching = set()
        self.right_touching = set()
        self.left_contact_stamp = 0.0
        self.right_contact_stamp = 0.0
        self.grasp_candidate = None
        self.grasp_since = None

    def check_grasp_condition(self):
        """
        Welds an object once both fingers have held it for GRASP_DWELL_TIME.

        The dwell is the whole point: the DetachableJoint captures the object's pose
        relative to base_link at the instant it attaches, so welding on the first
        contact frame freezes whatever glancing, half-off-the-fingertip pose that
        frame happened to have - which is what left props hanging rigidly in mid-air
        beside the vehicle. Waiting a second lets the fingers squeeze the prop down
        into the jaw first, so the pose that gets captured is a settled one.
        """
        with self.gripper_lock:
            # Already holding something, or the fingers were never told to close.
            if self.attached_object is not None or not self.gripper_commanded_closed:
                self.grasp_candidate = None
                self.grasp_since = None
                return

            now = self.current_time
            if now <= 0.0:
                # No /clock yet, so no trustworthy dwell. Never weld on a blind guess.
                return

            left, right = self.live_touching(now)
            candidates = {
                obj for obj in left.intersection(right)
                if obj not in IGNORED_CONTACT_MODELS
            }

            if not candidates:
                if self.grasp_candidate is not None:
                    self.get_logger().info(
                        f'Contact with {self.grasp_candidate} broke after '
                        f'{now - self.grasp_since:.2f}s; dwell reset.')
                self.grasp_candidate = None
                self.grasp_since = None
                return

            # Prefer something we can actually weld, and stay with the object already
            # being timed so brushing a second prop does not restart the clock.
            if self.grasp_candidate in candidates:
                obj = self.grasp_candidate
            else:
                weldable = sorted(c for c in candidates if c in self.attachable_objects)
                obj = weldable[0] if weldable else sorted(candidates)[0]

            if obj != self.grasp_candidate:
                self.grasp_candidate = obj
                self.grasp_since = now
                self.get_logger().info(
                    f'Both fingers on {obj}; holding {GRASP_DWELL_TIME}s before grasping.')
                return

            if now - self.grasp_since < GRASP_DWELL_TIME:
                return

            # Held long enough. Publish the name of the held object to your ROS 2 stack
            msg = String()
            msg.data = obj
            self.grasped_object_pub.publish(msg)

            self.get_logger().info(
                f'Grasp confirmed on: {obj} after {now - self.grasp_since:.2f}s of contact.')

            # Weld it to base_link so it rides with the AUV
            self.attach_object(obj)

    def gripper_joint_state_callback(self, obj, msg):
        """ Gazebo reports 'attached'/'detached' whenever obj's DetachableJoint changes. """
        state = msg.data.strip().lower()

        if state == 'detached':
            self.joints_released.add(obj)
            with self.gripper_lock:
                if self.attached_object == obj:
                    self.get_logger().info(f'Gazebo released {obj}.')
                    self.attached_object = None
        elif state == 'attached':
            self.get_logger().info(f'Gripper detachable joint for {obj} is attached.')

    def release_initial_attachment(self):
        """
        Every DetachableJoint declared in model.sdf is welded from world load because
        gz-sim 8 has no <suppress_initial_attach>. Keep asking Gazebo to open all of
        them until the attempt cap, so each object starts life free.

        This cannot stop as soon as the first joint reports open: a world carries only
        some of the five, and the ones it does carry do not all confirm on the same
        tick. Publishing detach at an already-open joint is a no-op, so running the
        full 10 s is harmless and is the only way to be sure none is left welded.
        """
        if self.release_attempts >= 100:
            self.initial_release_timer.cancel()
            self.initial_release_done = True
            if self.joints_released:
                self.get_logger().info(
                    'Gripper detachable joints released at startup: '
                    f'{sorted(self.joints_released)}.')
            else:
                self.get_logger().info(
                    'No gripper detachable joint reported by Gazebo; nothing to release '
                    '(expected if this world has no graspable object).')
            return

        self.release_attempts += 1
        # Under the lock: handle_close_gripper runs in the default callback group, so
        # it can set attached_object while this timer is midway through the loop.
        with self.gripper_lock:
            held = self.attached_object
        for obj in self.attachable_objects:
            # Never re-open a joint the gripper has deliberately closed on. Without
            # this the startup loop would rip a payload out of the gripper if the
            # operator grabbed something inside the first 10 seconds.
            if obj == held:
                continue
            self.gripper_detach_pubs[obj].publish(EmptyMsg())

    def attach_object(self, obj):
        """ Triggers the DetachableJoint plugin so the object rigidly follows base_link. """
        with self.gripper_lock:
            if self.attached_object is not None:
                return

            # Contact alone is not a grasp - the fingers must have been commanded shut.
            if not self.gripper_commanded_closed:
                return

            if obj not in self.attachable_objects:
                self.get_logger().warn(
                    f'Grasped "{obj}" but no DetachableJoint is declared for it in model.sdf; '
                    'not attaching.',
                    throttle_duration_sec=5.0,
                )
                return

            # Set this before publishing: the startup release loop skips whatever is
            # recorded here, so it must already be recorded when the loop next runs.
            self.attached_object = obj
            self.gripper_attach_pubs[obj].publish(EmptyMsg())
            self.get_logger().info(f'Attached {obj} to the gripper.')

    def release_object(self):
        """ Drops whatever the gripper is holding. """
        with self.gripper_lock:
            if self.attached_object is None:
                return

            self.get_logger().info(f'Releasing {self.attached_object}.')
            self.gripper_detach_pubs[self.attached_object].publish(EmptyMsg())
            self.attached_object = None
            # The fingers were inside the object's geometry while it was welded, so
            # whatever the touch sets hold now describes a grasp that no longer exists.
            self.reset_grasp_tracking()

    def handle_drop_marker(self, side, response):
        """Release one marker. Each dropper fires once, like the real vehicle."""
        if side in self.markers_dropped:
            self.get_logger().warn(f'Marker {side} has already been dropped.')
            return response
        self.markers_dropped.add(side)
        self.marker_drop_pubs[side].publish(EmptyMsg())
        self.get_logger().info(f'Dropped marker {side}.')
        return response

    def handle_close_gripper(self, request, response):
        """ Closes both fingers to GRIPPER_CLOSE_ANGLE and arms grasping """
        # Arm first: the fingers may reach the object before this call returns. This also
        # cancels any release still pending from a recent open.
        with self.gripper_lock:
            self.gripper_commanded_closed = True

        left_cmd = Float64()
        left_cmd.data = -GRIPPER_CLOSE_ANGLE

        right_cmd = Float64()
        right_cmd.data = GRIPPER_CLOSE_ANGLE

        self.left_finger_pub.publish(left_cmd)
        self.right_finger_pub.publish(right_cmd)

        self.get_logger().info('Gripper closed.')

        # Start the dwell now rather than on the next contact message. This can only
        # ever begin timing - the weld itself is GRASP_DWELL_TIME away, and the poll
        # timer re-checks throughout, so a stale set cannot short-circuit it.
        self.check_grasp_condition()
        return response

    def handle_open_gripper(self, request, response):
        """ Opens both fingers back to 0.0 radians, then drops any held object """
        # Disarm immediately so a contact callback racing us cannot re-attach, and drop
        # the dwell so the next close starts its second from scratch rather than
        # inheriting contact recorded before the fingers opened.
        with self.gripper_lock:
            self.gripper_commanded_closed = False
            held = self.attached_object
            self.reset_grasp_tracking()

        left_cmd = Float64()
        left_cmd.data = 0.0

        right_cmd = Float64()
        right_cmd.data = 0.0

        self.left_finger_pub.publish(left_cmd)
        self.right_finger_pub.publish(right_cmd)

        if held is None:
            self.get_logger().info('Gripper opened.')
            return response

        # Retract the fingers BEFORE breaking the weld. A welded object is moved into
        # the AUV's own skeleton, and <self_collide> defaults to false, so while it is
        # held the fingers pass straight through it and settle inside its geometry.
        # Detaching at that instant hands the physics engine two deeply overlapping
        # bodies, which it resolves by flicking the object out sideways or wedging it on
        # a finger. Letting the fingers reach 0 rad first means the object is in free
        # space by the time it becomes a separate body again.
        self.deferred_release_timer.reset()
        self.get_logger().info(
            f'Gripper opening; releasing {held} in {FINGER_RETRACT_TIME}s.')
        return response

    def deferred_release(self):
        """ Second half of handle_open_gripper, once the fingers have cleared. """
        self.deferred_release_timer.cancel()

        with self.gripper_lock:
            if self.gripper_commanded_closed:
                # close_gripper was called again while we were waiting - keep holding.
                self.get_logger().info('Release cancelled; gripper was closed again.')
                return
            self.release_object()


def main(args=None):
    rclpy.init(args=args)
    # Specified 8 threads to ensure ample availability for separated groups
    executor = MultiThreadedExecutor(num_threads=8)
    simulator_bridge = SimulatorBridge()
    executor.add_node(simulator_bridge)
    executor.spin()
    simulator_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()