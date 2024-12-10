#!/usr/bin/env python

import numpy as np
import open3d as o3d
from nuscenes.nuscenes import NuScenes
from pyquaternion import Quaternion
from nuscenes.utils.geometry_utils import points_in_box
import time
from nuscenes.utils.data_classes import Box
import matplotlib.pyplot as plt
from camera_pc_utils import *


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
    
    return pc[:, :3]                                                                                    # x, y, z only

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
    annotations = []
    pc_colors = []
    idx = 1 # Scene index
    sample_token = nusc.scene[idx]['first_sample_token']  
    print(f"Scene {idx}: {nusc.scene[idx]['name']}")   
    while sample_token:
        sample = nusc.get('sample', sample_token)
        lidar_data_token    = sample['data']['LIDAR_TOP']

        # Get lidar sample data
        lidar_data = nusc.get('sample_data', lidar_data_token)        

        lidar_calib = nusc.get('calibrated_sensor', lidar_data['calibrated_sensor_token'])
        lidar_R     = lidar_calib['rotation']                                                       # Quaternion [w, x, y, z]
        lidar_t     = np.array(lidar_calib['translation'])                                          # [x, y, z]

        # Load the point cloud
        pc  = load_lidar_point_cloud(lidar_data_token)

        # Get the ego pose in global coordinate frame
        ego_pose    =   nusc.get('ego_pose', lidar_data['ego_pose_token'])
        ego_t       =   np.array(ego_pose['translation'])                                               # [x, y, z]
        ego_R       =   ego_pose['rotation']                                                            # Quaternion [w, x, y, z]

        # Transform the point cloud to global coordinate frame using ego pose
        pc_global   = transform_point_cloud(pc, lidar_t, lidar_R, ego_t, ego_R)
        cam_data_tokens = [
            sample['data'][f'CAM_{camera}'] for camera in 
            ['FRONT', 
             'FRONT_LEFT', 
             'FRONT_RIGHT', 
             'BACK', 
             'BACK_LEFT', 
             'BACK_RIGHT']
             ]
        
        _, colors = enhance_point_cloud_with_colors(pc_global, cam_data_tokens, nusc)

        pc_colors.append(colors)
        aggregated_pc.append(pc_global)

        for ann_token in sample['anns']:
            ann = nusc.get('sample_annotation', ann_token)
            annotations.append(ann)

        sample_token = sample['next']

    return aggregated_pc, annotations, pc_colors

def draw_pc(pc, pc_colors):
    """
    Visualize the point cloud
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc)
    pcd.colors = o3d.utility.Vector3dVector(pc_colors)
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Point Cloud")
    vis.get_render_option().background_color = np.array([0, 0, 0])                                      # Black background
    vis.add_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    vis.run()
    vis.destroy_window()

def visualize_pc(pc_sequence, delay=0.1):
    """
    Visualize a sequence of point clouds to simulate motion.
    
    Parameters:
    - pc_sequence: List of np.ndarray, where each element is a point cloud (N x 3).
    - delay: Time delay (in seconds) between frames, default is 0.1s.
    """
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_sequence[0])
    vis.add_geometry(pcd)
    accumlated_pc = np.array(pc_sequence[0])

    # Iterate through the point cloud sequence
    for pc in pc_sequence[1:]:

        accumlated_pc = np.vstack((accumlated_pc, pc))

        # Update the point cloud with new data
        pcd.points = o3d.utility.Vector3dVector(accumlated_pc)
        vis.update_geometry(pcd)

        # Render the frame and introduce a delay to simulate motion
        vis.poll_events()
        vis.update_renderer()
        time.sleep(delay)
    vis.clear_geometries()
    vis.destroy_window()


def detect_moving_objects(agg_pc, annotations, v_t = 0.25):
    """
    Identify points corresponding to moving objects in the aggregated point cloud.

    Parameters:
        aggregated_pc (np.ndarray): Aggregated global point cloud (Nx3).
        annotations (list): List of annotations for the scene.
        velocity_threshold (float): Minimum velocity to consider an object as moving.

    Returns:
        moving_points (np.ndarray): Points corresponding to moving objects.
        static_points (np.ndarray): Points corresponding to static objects.
    """

    # Masks for moving and static points
    moving_mask = np.zeros(agg_pc.shape[0], dtype=bool)
    static_mask = np.ones(agg_pc.shape[0], dtype=bool)

    for ann in annotations:

        # Get velocity
        velocity = get_velocity(ann)
        is_moving = velocity > v_t

        # Bounding box for object
        box = Box(
        ann['translation'],
        ann['size'],
        Quaternion(ann['rotation'])
        )
        # Get inside box points
        points_inside_box = points_in_box(box, agg_pc.T)

        if is_moving and points_inside_box.sum() > 30:
            moving_mask[points_inside_box] = True
        else:
            static_mask[points_inside_box] = False

    moving_points = agg_pc[moving_mask]
    static_points = agg_pc[static_mask]

    return moving_points, static_points


def visualize_static_and_moving_points(static_points, pc_colors, moving_points=[]):
    """
    Visualize moving and static points using Open3D.

    Parameters:
        static_points (np.ndarray): Points corresponding to static objects (Nx3).
        moving_points (np.ndarray): Points corresponding to moving objects (Nx3).
    """
    # Create Open3D point clouds
    moving_pcd          =   o3d.geometry.PointCloud()
    static_pcd          =   o3d.geometry.PointCloud()
    moving_pcd.points   =   o3d.utility.Vector3dVector(moving_points)
    static_pcd.points   =   o3d.utility.Vector3dVector(static_points)

    # Assign color for moving object: Red
    moving_pcd.paint_uniform_color([1, 0, 0])  # Red

    # Visualize moving and static points
    o3d.visualization.draw_geometries([moving_pcd, static_pcd])


def get_velocity(ann):
    if 'velocity' in ann:
        return np.linalg.norm(ann['velocity'], ord=2)
    elif ann['prev']:
        # Calculate velocity using previous annotation
        current_translation =   np.array(ann['translation'])
        current_timestamp   =   nusc.get('sample', ann['sample_token'])['timestamp']
        prev_ann            =   nusc.get('sample_annotation', ann['prev'])
        prev_translation    =   np.array(prev_ann['translation'])
        prev_timestamp      =   nusc.get('sample', prev_ann['sample_token'])['timestamp']
        displacement        =   current_translation - prev_translation
        time_delta          =   (current_timestamp - prev_timestamp) / 1e6                                  # Convert microseconds to seconds
        return np.linalg.norm(displacement) / time_delta
    else:
        # No velocity info or no previous annotation
        return 0.0


def show_scene_image():
    # Scene selection
    scene_name = "scene-0103"
    scene = next(s for s in nusc.scene if s["name"] == scene_name)

    # Iterate through samples in the scene
    current_sample_token = scene['first_sample_token']
    while current_sample_token:
        # Load the sample
        sample = nusc.get('sample', current_sample_token)

        # Camera data
        for cam in ['CAM_FRONT', 'CAM_FRONT_LEFT']:
            cam_token = sample['data'][cam]
            cam_data = nusc.get('sample_data', cam_token)
            cam_file_path = nusc.get_sample_data_path(cam_token)

            # Visualize the camera image (optional)
            img = plt.imread(cam_file_path)
            plt.imshow(img)
            plt.title(cam)
            plt.show()
        current_sample_token = sample['next']
