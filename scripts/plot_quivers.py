import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

def load_poses(json_file):
    with open(json_file, 'r') as f:
        data = json.load(f)
    frames = data['frames']
    poses = [frame['transform_matrix'] for frame in frames]
    
    return poses

def apply_rotations(poses):
    # Rotation matrices using scipy
    Ry_90 = R.from_euler('y', 90, degrees=True).as_matrix()
    Rx_90 = R.from_euler('x', -90, degrees=True).as_matrix()
    
    rotated_poses = []
    for pose in poses:
        transform_matrix = np.array(pose)
        
        # Extract the rotation part and the translation part
        rotation_matrix = transform_matrix[:3, :3]
        translation_vector = transform_matrix[:3, 3]
        
        # Apply rotations
        rotated_matrix = rotation_matrix @ Ry_90
        rotated_matrix = rotated_matrix @ Rx_90
        # rotated_matrix = rotation_matrix
        
        # Combine the rotated matrix with the translation part
        new_transform_matrix = np.eye(4)
        new_transform_matrix[:3, :3] = rotated_matrix
        new_transform_matrix[:3, 3] = translation_vector
        
        rotated_poses.append(new_transform_matrix.tolist())
    
    return rotated_poses

def plot_poses(poses):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    for pose in poses:
        transform_matrix = np.array(pose)
        position = transform_matrix[:3, 3]
        rotation_matrix = transform_matrix[:3, :3]
        
        # Define the origin of the coordinate frame
        origin = position
        
        # Define the x, y, z unit vectors
        x_dir = rotation_matrix[:, 0]
        y_dir = rotation_matrix[:, 1]
        z_dir = rotation_matrix[:, 2]
        
        # Plot the coordinate frame
        ax.quiver(origin[0], origin[1], origin[2], x_dir[0], x_dir[1], x_dir[2], color='r', length=0.1, normalize=True)
        ax.quiver(origin[0], origin[1], origin[2], y_dir[0], y_dir[1], y_dir[2], color='g', length=0.1, normalize=True)
        ax.quiver(origin[0], origin[1], origin[2], z_dir[0], z_dir[1], z_dir[2], color='b', length=0.1, normalize=True)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Poses Visualization')
    
    plt.show()


def main():
    json_file = 'testing/transforms.json'
    # json_file = 'testing/pose_data.json'
    poses = load_poses(json_file)
    rotated_poses = apply_rotations(poses)
    reduced_poses = poses[::3]
    # plot_poses(reduced_poses)
    # plot_poses(poses)
    plot_poses(rotated_poses)

if __name__ == '__main__':
    main()
