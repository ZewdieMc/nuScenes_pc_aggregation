#!/usr/bin/env python

from nuscenes.nuscenes import NuScenes

# Initialize NuScenes
nusc = NuScenes(version='v1.0-mini', dataroot='data/sets/nuscenes', verbose=True)

# Access a sample
sample = nusc.get('sample', nusc.sample[0]['token'])
print("Total number of samples: ", len(nusc.sample))
print("sample[data] keys: ", sample['data'].keys())
print("total samples: ", len(nusc.sample))
# Lidar calibration
lidar_data = nusc.get('sample_data', sample['data']['LIDAR_TOP'])
calibrated_sensor = nusc.get('calibrated_sensor', lidar_data['calibrated_sensor_token'])
# print("Lidar data keys: ", calibrated_sensor.keys())
print("Lidar calibration: ", calibrated_sensor['translation'], calibrated_sensor['rotation'])

# Access ego motion
ego_pose = nusc.get('ego_pose', lidar_data['ego_pose_token'])
print("Ego pose: ", ego_pose['translation'], ego_pose['rotation'])

for (i, sample) in enumerate(nusc.sample):
    sample_data = nusc.get('sample_data', sample['data']['LIDAR_TOP'])
    c_s  = nusc.get('calibrated_sensor', sample_data['calibrated_sensor_token'])
    e_p = nusc.get('ego_pose', sample_data['ego_pose_token'])
    print(f"T {e_p['translation']} R {e_p['rotation']}")
    if i == 5:
        print(f"T {c_s['translation']} R {c_s['rotation']}")
        break