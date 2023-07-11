#!/usr/bin/env python
# -*-coding:utf-8-*-

# ROS
import ros_numpy as rosnp
import numpy as np
from tf import transformations
import std_msgs.msg
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d

import time

class TransPointCloud:
    def __init__(self):
        self._xyz = None # x, y, z
        self._euler_ang = None # roll, picth, yaw
        self._frame_id = None
        self._sampling_ratio = 0.05
        self._distance_threshold = 0.01
        self._ransac_n = 100
        self._num_iterations = 100
    
    def transform_matrix(self, trans, euler_ang):
        self._xyz = np.array(trans)
        self._euler_ang = np.array(euler_ang)
        Rotation_mat = transformations.euler_matrix(self._euler_ang[0], self._euler_ang[1], self._euler_ang[2], 'sxyz')
        translation_mat = np.insert(self._xyz, 3, 1, axis=0)
        Transform_mat = Rotation_mat
        Transform_mat[:, 3] = translation_mat

        return Transform_mat
    
    # Transform point cloud from current frame to target frame
    # Return point cloud as NumPy Array
    def trans_pcd_ground(self, trans_mat, point_cloud_msg, frame_id, mode):
        self._frame_id = frame_id

        
        point_cloud_ref = self.pc2array_surf(point_cloud_msg)
        point_cloud_tar = np.dot(trans_mat, point_cloud_ref)

        if mode == 0:
            return point_cloud_tar
        
        if mode == 1:
            point_cloud_tar_msg = self.array2pc(point_cloud_msg, point_cloud_tar)
            return point_cloud_tar_msg
        
        if mode == 2:
            point_cloud_tar_msg = self.array2pc(point_cloud_msg, point_cloud_tar)
            return point_cloud_tar, point_cloud_tar_msg
    
    
    # Transform point cloud from current frame to target frame
    # Return point cloud as NumPy Array
    def trans_pcd(self, trans, euler_ang, point_cloud_msg, frame_id, mode):
        self._frame_id = frame_id
        trans_mat = self.transform_matrix(trans, euler_ang)

        point_cloud_ref = self.pc2array(point_cloud_msg)
        point_cloud_tar = np.dot(trans_mat, point_cloud_ref)

        if mode == 0:
            return point_cloud_tar
        
        if mode == 1:
            point_cloud_tar_msg = self.array2pc(point_cloud_msg, point_cloud_tar)
            return point_cloud_tar_msg
        
        if mode == 2:
            point_cloud_tar_msg = self.array2pc(point_cloud_msg, point_cloud_tar)
            return point_cloud_tar, point_cloud_tar_msg

    # Transform downsampled point cloud from current frame to target frame
    # Return point cloud as NumPy Array  
    def trans_pcd_dsamp(self, trans, euler_ang, point_cloud_msg, frame_id, mode):
        self._frame_id = frame_id
        trans_mat = self.transform_matrix(trans, euler_ang)

        point_cloud_ref = self.pc2array_ds(point_cloud_msg)
        point_cloud_ref_down = self.down_sample(point_cloud_ref)
        ones = np.ones((1, point_cloud_ref_down.shape[0]))
        point_cloud_ref_down = np.concatenate([point_cloud_ref_down.T, ones], axis=0)
        point_cloud_tar = np.dot(trans_mat, point_cloud_ref_down)

        if mode == 0:
            return point_cloud_tar
        if mode == 1:
            point_cloud_tar_msg = self.array2pc(point_cloud_msg, point_cloud_tar)
            return point_cloud_tar_msg
        if mode == 2:
            point_cloud_tar_msg = self.array2pc(point_cloud_msg, point_cloud_tar)
            return point_cloud_tar, point_cloud_tar_msg

    # Segment ground from downsampled point cloud
    # Transform the ground points from current frame to target frame
    # Return a NumPy Array to represent ground
    def seg_ground(self, trans, euler_ang, point_cloud_msg, frame_id, mode):
        self._frame_id = frame_id
        trans_mat = self.transform_matrix(trans, euler_ang)

        # start1 = time.time()
        point_cloud_ref = self.pc2array_ds(point_cloud_msg)
        point_cloud_ref_down = self.down_sample_o3d(point_cloud_ref)
        ground_ref = self.ground_segmentation(point_cloud_ref_down)

        ones = np.ones((1, ground_ref.shape[0]))
        ground_ref = np.concatenate([ground_ref.T, ones], axis=0)
        point_cloud_tar = np.dot(trans_mat, ground_ref)
        # end1 = time.time()
        # time1 = end1 - start1
        # print('Ground Seg Time', time1)

        if mode == 0:
            return point_cloud_tar
        if mode == 1:
            point_cloud_tar_msg = self.array2pc(point_cloud_msg, point_cloud_tar)
            return point_cloud_tar_msg
        if mode == 2:
            # start1 = time.time()
            point_cloud_tar_msg = self.array2pc(point_cloud_msg, point_cloud_tar)
            # end1 = time.time()
            # time1 = end1 - start1
            # print('Array2PC Time', time1)
            return point_cloud_tar, point_cloud_tar_msg

    def pc2array_surf(self, point_cloud_msg):
        pc = rosnp.numpify(point_cloud_msg)
        # print(np.shape(pc))
        # print(pc)
        pc_array = np.ones((4, pc.shape[0]))
        pc_array[0, :]=pc['x']
        pc_array[1, :]=pc['y']
        pc_array[2, :]=pc['z']

        return pc_array

    def pc2array(self, point_cloud_msg):
        pc = rosnp.numpify(point_cloud_msg)
        num = pc.shape[0]*pc.shape[1]
        pc = np.reshape(pc, [1, num])
        pc_array = np.ones((4, num))
        pc_array[0, :]=pc['x']
        pc_array[1, :]=pc['y']
        pc_array[2, :]=pc['z']

        return pc_array

    def pc2array_ds(self, point_cloud_msg):
        pc = rosnp.numpify(point_cloud_msg)
        num = pc.shape[0]*pc.shape[1]
        pc = np.reshape(pc, [1, num])
        pc_array = np.zeros((num, 3))
        pc_array[:, 0]=pc['x']
        pc_array[:, 1]=pc['y']
        pc_array[:, 2]=pc['z']

        return pc_array

    def down_sample(self, pc_array_xyz):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc_array_xyz)
        # downpcd = o3d.geometry.random_down_sample(pcd, sampling_ratio=self._sampling_ratio)
        downpcd = pcd.voxel_down_sample(voxel_size=0.05)
        pc_array = np.asarray(downpcd.points)

        return pc_array
    
    def down_sample_o3d(self, pc_array_xyz):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc_array_xyz)
        # downpcd = o3d.geometry.random_down_sample(pcd, sampling_ratio=self._sampling_ratio)
        downpcd = pcd.voxel_down_sample(voxel_size=0.05)

        return downpcd

    def ground_segmentation(self, pcd):
        plane_model, inliers = pcd.segment_plane(distance_threshold=self._distance_threshold,
                                                ransac_n=self._ransac_n,
                                                num_iterations=self._num_iterations)
        [a, b, c, d] = plane_model
        # print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
        # print([a, b, c, d])

        inlier_cloud = pcd.select_down_sample(inliers)
        # print('----inlier_cloud: ', inlier_cloud.points)
        outlier_cloud = pcd.select_down_sample(inliers, invert=True)
        # print('----outlier_cloud: ', outlier_cloud.points)

        pc_array = np.asarray(inlier_cloud.points)

        return pc_array


    def array2pc(self, point_cloud_msg, point_cloud_array):
        pc = rosnp.numpify(point_cloud_msg)
        points = point_cloud_array.T
        points = np.delete(points, 3, axis=1)
        zeros = np.zeros((points.shape[0], 3))
        points = np.concatenate((points, zeros), axis = 1)

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),
            PointField('ring', 20, PointField.FLOAT32, 1),
            PointField('timestamp', 24, PointField.FLOAT32, 1)]

        header = std_msgs.msg.Header()
        header.seq = point_cloud_msg.header.seq
        header.stamp = point_cloud_msg.header.stamp
        header.frame_id = self._frame_id

        pc = pc2.create_cloud(header = header, fields = fields, points = points)
        
        return pc