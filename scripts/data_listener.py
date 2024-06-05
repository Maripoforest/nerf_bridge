import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import json
import os
from scipy.spatial.transform import Rotation as R
import numpy as np

class PoseRecorder(Node):
    def __init__(self):
        super().__init__('pose_recorder')
        self.pose_sub = self.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry',
            self.pose_callback,
            10)
        self.data = {'frames': []}

    def pose_callback(self, msg):
        # Extract the pose (transform_matrix)
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        transform_matrix = self.pose_to_transform_matrix(position, orientation)
        
        # Record the pose data
        frame_data = {
            'transform_matrix': transform_matrix.tolist()
        }
        self.data['frames'].append(frame_data)
    
    def pose_to_transform_matrix(self, position, orientation):
        # Convert position and orientation to a 4x4 transformation matrix
        rotation = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation.as_matrix()
        transform_matrix[0, 3] = position.x
        transform_matrix[1, 3] = position.y
        transform_matrix[2, 3] = position.z
        return transform_matrix

    def save_data(self):
        with open('testing/pose_data.json', 'w') as f:
            json.dump(self.data, f, indent=4)

def main(args=None):
    rclpy.init(args=args)
    node = PoseRecorder()
    print("Start")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.save_data()
    print("Finished")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
