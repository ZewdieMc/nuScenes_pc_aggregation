#!/usr/bin/env python

import numpy as np
import open3d as o3d
from nuscenes.nuscenes import NuScenes
from pyquaternion import Quaternion

# Initialize NuScenes devkit
DATA_ROOT = 'data/sets/nuscenes'
nusc = NuScenes(version='v1.0-mini', dataroot=DATA_ROOT, verbose=True)

def load_lidar_point_cloud(sample_data_token):
    """
    Load the lidar point cloud from the sample_data token
    """
    lidar_data = nusc.get('sample_data', sample_data_token)
    ld_file_path = f"{DATA_ROOT}/{lidar_data['filename']}"
    pc = np.fromfile(ld_file_path, dtype=np.float32).reshape(-1, 5)#o3d.io.read_point_cloud(ld_file_path)
    return pc[:, :3] # x, y, z only

def transform_point_cloud(pc, lidar_t, lidar_R, ego_t, ego_R):
    """
    Transform the point cloud from the lidar frame to the global frame.
    
    Args:
    - points: Point cloud in the lidar frame (Nx3 array of x, y, z).
    - sensor_translation: Translation of the lidar sensor relative to the vehicle frame.
    - sensor_rotation: Rotation of the lidar sensor relative to the vehicle frame (quaternion).
    - ego_translation: Translation of the vehicle frame relative to the global frame.
    - ego_rotation: Rotation of the vehicle frame relative to the global frame (quaternion).
    """

    ld_R_mat = Quaternion(lidar_R).rotation_matrix
    pc_vehicle = np.dot(pc, ld_R_mat.T) + lidar_t

    # convert quaterion to rotation matrix
    ego_R_mat = Quaternion(ego_R).rotation_matrix

    # apply rotation and translation
    pc_global = np.dot(pc_vehicle, ego_R_mat.T) + ego_t
    return pc_global


def aggregate_pc():
    """
    Aggregate point clouds into a global coordinate system
    """

    aggregated_pc = []
    print(len(nusc.sample))
    for sample in nusc.sample:
        # Get lidar sample data
        lidar_data_token = sample['data']['LIDAR_TOP']
        lidar_data = nusc.get('sample_data', lidar_data_token)

        lidar_calib = nusc.get('calibrated_sensor', lidar_data['calibrated_sensor_token'])
        lidar_R = lidar_calib['rotation'] # Quaternion [w, x, y, z]
        lidar_t = np.array(lidar_calib['translation']) # [x, y, z]

        # Load the point cloud
        pc  = load_lidar_point_cloud(lidar_data_token)

        # Get the ego pose in global coordinate frame
        ego_pose = nusc.get('ego_pose', lidar_data['ego_pose_token'])
        ego_t = np.array(ego_pose['translation']) # [x, y, z]
        ego_R = ego_pose['rotation'] # Quaternion [w, x, y, z]

        # Transform the point cloud to global coordinate frame using ego pose
        pc_global = transform_point_cloud(pc, lidar_t, lidar_R, ego_t, ego_R)

        aggregated_pc.append(pc_global)
    return np.vstack(aggregated_pc)

def visualize_pc(pc):
    """
    Visualize the point cloud
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc)
    o3d.visualization.draw_geometries([pcd])