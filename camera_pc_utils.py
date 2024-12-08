from nuscenes.nuscenes import NuScenes
import numpy as np
from pyquaternion import Quaternion
import matplotlib.pyplot as plt


def project_points_to_image(pc:np.ndarray, cam_intrinsic: np.ndarray, cam_extrinsic: np.ndarray) -> np.ndarray:
    """
    Project 3D points to 2D image plane.

    Parameters:
        points (np.ndarray): 3D points (Nx3).
        cam_intrinsic (np.ndarray): Camera intrinsic matrix (3x3).
        cam_extrinsic (np.ndarray): Camera extrinsic matrix (4x4).

    Returns:
        np.ndarray: 2D points (Nx2).
    """

    # convert points to homogenous coordinates
    pc_h = np.hstack((pc, np.ones((pc.shape[0], 1))))

    # Transform to camera coordinate systems
    pc_cam = (cam_extrinsic @ pc_h.T).T

    # Apply intrinsic transformation
    pc_img = (cam_intrinsic @ pc_cam[:, :3].T).T

    # Normalize to get pixel cooridnates
    pc_img = pc_img[:, :2] / pc_img[:, 2:]

    return pc_img
def get_color_from_image(pc_img:np.ndarray, image):
    """
    Get color values from the image for the projected 2D points.

    Parameters:
        points_img (np.ndarray): 2D points (Nx2).
        image (np.ndarray): Image data.

    Returns:
        np.ndarray: Color values (Nx3).
    """
    h, w, _ = image.shape
    colors = []

    for pt in pc_img:
        x, y = int(pt[0]), int(pt[1])
        if 0 <= x < w and 0 <= y < h:
            colors.append(image[y, x, :])
        else:
            colors.append([0, 0, 0])                                                        # Default color for out-of-bound points

    return np.array(colors) / 255.0                                                         # Normalize to [0, 1]


def enhance_point_cloud_with_colors(static_points, cam_data_tokens, nusc):
    """
    Enhance the static point cloud with color values from the camera images.

    Parameters:
        static_points (np.ndarray): Static points (Nx3).
        cam_data_tokens (list): List of camera data tokens.

    Returns:
        o3d.geometry.PointCloud: Enhanced point cloud with colors.
    """
    colors = np.zeros((static_points.shape[0], 3))
    distances = np.full(static_points.shape[0], np.inf)

    for cam_data_token in cam_data_tokens:                                                 #cam_data_tokens = [sample['data']['xxxx']]
        # Get camera data and calibration
        cam_data = nusc.get('sample_data', cam_data_token)
        cam_calib = nusc.get('calibrated_sensor', cam_data['calibrated_sensor_token'])
        cam_intrinsic = np.array(cam_calib['camera_intrinsic'])
        cam_extrinsic = np.eye(4)

        # compute extrinsic from calibration and ego pose
        vTc         = np.eye(4)                                                            # camera in vehicle frame    
        vTc[:3, :3] =   Quaternion(cam_calib['rotation']).rotation_matrix
        vTc[:3, 3]  =   cam_calib['translation']

        wTv         =   np.eye(4)                                                          # vehicle in global frame
        ego_pose    =   nusc.get("ego_pose", cam_data['ego_pose_token'])
        ego_t       =   np.array(ego_pose['translation'])
        ego_R       =   ego_pose['rotation']

        wTv[:3, :3] =   Quaternion(ego_R).rotation_matrix
        wTv[:3, 3]  =   ego_t

        wTc            =    np.dot(wTv, vTc)                                               # camera in the global frame
        cam_extrinsic  =    np.linalg.inv(wTc)                                             # camera extrinsic    
        
        # Load the camera image
        img_path = nusc.get_sample_data_path(cam_data_token)
        image = plt.imread(img_path)

        # Project points to image plane
        points_img = project_points_to_image(static_points, cam_intrinsic, cam_extrinsic)

        # Get color values from the image
        cam_colors = get_color_from_image(points_img, image)

        # Calculate distances from camera to points
        cam_position = cam_extrinsic[:3, 3]
        point_distances = np.linalg.norm(static_points - cam_position, axis=1)

        # Update colors based on the closest camera view
        mask = (point_distances < distances) & (cam_colors != [0, 0, 0]).all(axis=1)
        colors[mask] = cam_colors[mask]
        distances[mask] = point_distances[mask]
        assert static_points.shape[0] == len(colors), "points and colors must have the same shape"
    return static_points, colors