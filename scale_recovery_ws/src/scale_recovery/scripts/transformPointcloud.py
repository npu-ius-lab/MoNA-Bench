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

    def pc2array_surf(self, point_cloud_msg):
        pc = rosnp.numpify(point_cloud_msg)
        pc_array = np.ones((4, pc.shape[0]))
        pc_array[0, :]=pc['x']
        pc_array[1, :]=pc['y']
        pc_array[2, :]=pc['z']

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