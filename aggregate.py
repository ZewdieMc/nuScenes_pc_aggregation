#!/usr/bin/env python

import numpy as np
import open3d as o3d
from nuscenes.nuscenes import NuScenes
from pyquaternion import Quaternion
import os 
import time

# Initialize NuScenes devkit
DATA_ROOT = 'data/sets/nuscenes'
nusc = NuScenes(version='v1.0-mini', dataroot=DATA_ROOT, verbose=False)

def load_lidar_point_cloud(sample_data_token):
    """
    Load the lidar point cloud from the sample_data token
    """
    lidar_data = nusc.get('sample_data', sample_data_token)
    ld_file_path = f"{DATA_ROOT}/{lidar_data['filename']}"
    pc = np.fromfile(ld_file_path, dtype=np.float32).reshape(-1, 5)
    
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
    for idx, scene in enumerate(nusc.scene):
        sample_token = scene['first_sample_token']

        while sample_token:
            sample = nusc.get('sample', sample_token)
            lidar_data_token = sample['data']['LIDAR_TOP']

            # Get lidar sample data
            lidar_data = nusc.get('sample_data', lidar_data_token)        

            lidar_calib = nusc.get('calibrated_sensor', lidar_data['calibrated_sensor_token'])
            lidar_R = lidar_calib['rotation']                                                       # Quaternion [w, x, y, z]
            lidar_t = np.array(lidar_calib['translation'])                                          # [x, y, z]

            # Load the point cloud
            pc  = load_lidar_point_cloud(lidar_data_token)

            # Get the ego pose in global coordinate frame
            ego_pose = nusc.get('ego_pose', lidar_data['ego_pose_token'])
            ego_t = np.array(ego_pose['translation'])                                               # [x, y, z]
            ego_R = ego_pose['rotation']                                                            # Quaternion [w, x, y, z]

            # Transform the point cloud to global coordinate frame using ego pose
            pc_global = transform_point_cloud(pc, lidar_t, lidar_R, ego_t, ego_R)
            if idx == 1:
                aggregated_pc.append(pc_global)
            sample_token = sample['next']

    return aggregated_pc

def draw_pc(pc):
    """
    Visualize the point cloud
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc)
    # o3d.visualization.draw_geometries([pcd])

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Point Cloud", width=800, height=600)
    vis.get_render_option().background_color = np.array([0, 0, 0])  # Black background
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()

def visualize_pc(pc_sequence, delay=0.2):
    """
    Visualize a sequence of point clouds to simulate motion.
    
    Parameters:
    - pc_sequence: List of np.ndarray, where each element is a point cloud (N x 3).
    - delay: Time delay (in seconds) between frames, default is 0.1s.
    """
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # vis.get_render_option().point_size = 0.05
    vis.get_render_option().background_color = np.array([0, 0, 0])

    # Add an initial point cloud geometry to the visualizer
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_sequence[0])
    vis.add_geometry(pcd)
    # !view_control = vis.get_view_control()


    # Iterate through the point cloud sequence
    for pc in pc_sequence[1:]:
        # Update the point cloud with new data
        pcd.points = o3d.utility.Vector3dVector(pc)
        vis.update_geometry(pcd)

        # !bbox = pcd.get_axis_aligned_bounding_box()
        # !view_control.set_lookat(bbox.get_center())
        # Render the frame and introduce a delay to simulate motion
        vis.poll_events()
        vis.update_renderer()
        time.sleep(delay)
    vis.clear_geometries()
    vis.destroy_window()

def visualize_moving_objects(pc):
    """
    Removes moving objects from the point cloud.

    Args:
    - pc: np.ndarray of shape (N, 3) representing the point cloud.
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc)

    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)

    # Extract inliers and outliers
    inlier_pcd = pcd.select_by_index(ind)
    outlier_pcd = pcd.select_by_index(ind, invert=True)

    outlier_pcd.paint_uniform_color([0, 0, 1])  # Red

    combined_pcd = inlier_pcd + outlier_pcd
    o3d.visualization.draw_geometries([combined_pcd])
    return combined_pcd