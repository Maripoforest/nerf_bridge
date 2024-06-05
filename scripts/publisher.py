import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge
import json
import os
import time
import numpy as np
from scipy.spatial.transform import Rotation as R


class ImagePosePublisher(Node):
    def __init__(self):
        super().__init__('image_pose_publisher')
        self.image_publisher = self.create_publisher(Image, '/camera/color/image', 10)
        self.odometry_publisher = self.create_publisher(Odometry, '/visual_slam/tracking/odometry', 10)
        self.bridge = CvBridge()

        # Load JSON file containing pose and image information
        json_file = './testing/fox/transforms.json'  # Update with the correct path
        self.poses, self.image_paths = self.load_poses(json_file)

        # Publish images with poses
        self.publish_image_with_pose()

    def load_poses(self, json_file):
        with open(json_file, 'r') as f:
            data = json.load(f)
        frames = data['frames']
        poses = [frame['transform_matrix'] for frame in frames]
        image_paths = [os.path.join(os.path.dirname(json_file), frame['file_path']) for frame in frames]
        return poses, image_paths

    def publish_image_with_pose(self):
        # Extracting the first pose

        for i in range(len(self.poses)):
            pose = self.poses[i]
            image_path = self.image_paths[i]

            # Load image
            if not os.path.exists(image_path):
                self.get_logger().warn(f"Image file not found: {image_path}")
                continue
            image = cv2.imread(image_path)
            image_msg = self.bridge.cv2_to_imgmsg(image)

            # Create Odometry message with all zero twist and identical timestamp
            odometry_msg = Odometry()
            odometry_msg.pose.pose.position.x = pose[0][3]
            odometry_msg.pose.pose.position.y = pose[1][3]
            odometry_msg.pose.pose.position.z = pose[2][3]
            pose_array = np.array(pose)
            rotation_matrix = pose_array[:3, :3]
            rotated_matrix = self.apply_rotation(rotation_matrix)
            orientation = R.from_matrix(rotated_matrix).as_quat()
            odometry_msg.pose.pose.orientation.x = orientation[0]
            odometry_msg.pose.pose.orientation.y = orientation[1]
            odometry_msg.pose.pose.orientation.z = orientation[2]
            odometry_msg.pose.pose.orientation.w = orientation[3]
            
            # Get current time
            current_time = self.get_clock().now().to_msg()
            image_msg.header.stamp = current_time
            odometry_msg.header.stamp = current_time

            # Publish image
            self.image_publisher.publish(image_msg)
            self.get_logger().info(f"Published image with timestamp: {current_time}")

            # Publish odometry_msg
            self.odometry_publisher.publish(odometry_msg)
            self.get_logger().info(f"Published odometry message with timestamp: {current_time}")

            time.sleep(1)

    def apply_rotation(self, rotation_matrix):
        Ry_90 = R.from_euler('y', 90, degrees=True).as_matrix()
        Rx_90 = R.from_euler('x', -90, degrees=True).as_matrix()
        rotated_matrix = rotation_matrix @ Ry_90
        rotated_matrix = rotated_matrix @ Rx_90

        return rotated_matrix



def main(args=None):
    rclpy.init(args=args)
    image_pose_publisher = ImagePosePublisher()
    try:
        rclpy.spin(image_pose_publisher)
    except KeyboardInterrupt:
        pass

    image_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
