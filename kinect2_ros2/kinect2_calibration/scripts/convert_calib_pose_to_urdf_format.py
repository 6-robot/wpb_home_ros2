#!/usr/bin/env python3
"""Convert IAI Kinect2 calib_pose.yaml to a fixed URDF joint (ROS2/Python3).

Adapted from krepa098/kinect2_ros2, commit 3bc7974, Apache-2.0.
"""
import argparse
import math
import cv2
import numpy as np


def read_calib_pose(path):
    storage = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise ValueError(f'Cannot open {path}')
    try:
        rotation = storage.getNode('rotation').mat()
        translation = storage.getNode('translation').mat()
    finally:
        storage.release()
    if rotation is None or rotation.shape != (3, 3) or translation is None or translation.size != 3:
        raise ValueError('Expected a 3x3 rotation and three translation values')
    if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
        raise ValueError('Calibration contains non-finite values')
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5):
        raise ValueError('Invalid rotation matrix')
    return rotation, translation.reshape(3)


def calc_xyz_rpy(rotation, translation):
    cy = math.hypot(rotation[0, 0], rotation[1, 0])
    if cy > 1e-8:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        yaw = 0.0
    pitch = math.atan2(-rotation[2, 0], cy)
    return translation, (roll, pitch, yaw)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-f', required=True, help='Path to calib_pose.yaml')
    args = parser.parse_args()
    try:
        xyz, rpy = calc_xyz_rpy(*read_calib_pose(args.f))
    except (ValueError, cv2.error) as error:
        parser.error(str(error))
    print('<joint name="kinect2_rgb_joint" type="fixed">')
    print('  <origin xyz="{}" rpy="{}"/>'.format(
        ' '.join(f'{v:.12g}' for v in xyz), ' '.join(f'{v:.12g}' for v in rpy)))
    print('  <parent link="kinect2_rgb_optical_frame"/>')
    print('  <child link="kinect2_ir_optical_frame"/>')
    print('</joint>')


if __name__ == '__main__':
    main()
