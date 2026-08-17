import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2 as cv
import os
FRAMES_DIR=os.path.expanduser('~/ros2_ws/src/Robosub_ROS2_2024/auv_simbridge/auv_simbridge/frames')
os.makedirs(FRAMES_DIR, exist_ok=True)
class VisionTestNode(Node):
    def __init__(self):
        super().__init__('vision_test_node')
        self.front_camera_sub = self.create_subscription(
            Image,
            '/model/auv/front_camera/image',
            self.image_callback,
            10
        )
        self.front_camera_depth_sub = self.create_subscription(
            PointCloud2,
            '/model/auv/front_camera/points',
            self.depth_callback,
            10
        )
        self.bottom_camera_sub = self.create_subscription(
            Image,
            '/model/auv/bottom_camera/image',
            self.bottom_image_callback,
            10
        )
        self.bridge = CvBridge()
        self.counter=0

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        
        # Display the image using OpenCV
        time=self.get_clock().now().nanoseconds
        if self.counter%10==0:  # Save every 10th frame to reduce storage usage
            cv.imwrite(os.path.join(FRAMES_DIR, f'front_camera_image_{time}.jpg'), cv_image)  # Save the image to a file
        cv.imshow('Front Camera', cv_image)
        cv.waitKey(1)  # Add a small delay to allow the image to be displayed
        self.counter += 1

    def depth_callback(self, msg):
        # Process the PointCloud2 message as needed
        pass
    
    def bottom_image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Display the image using OpenCV
        time=self.get_clock().now().nanoseconds
        if self.counter%10==0:  # Save every 10th frame to reduce storage usage
            cv.imwrite(os.path.join(FRAMES_DIR, f'bottom_camera_image_{time}.jpg'), cv_image)  # Save the image to a file
        cv.imshow('Bottom Camera', cv_image)
        cv.waitKey(1)  # Add a small delay to allow the image to be displayed
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    vision_test_node = VisionTestNode()
    rclpy.spin(vision_test_node)
    vision_test_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':  
    main()