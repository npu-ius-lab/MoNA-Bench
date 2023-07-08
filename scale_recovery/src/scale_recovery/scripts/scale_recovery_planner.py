#!/usr/bin/env python
# -*-coding:utf-8-*-

# ROS
import cv2
import math
import rospy
import numpy as np
import message_filters
from cv_bridge import CvBridge
from transformPointcloud import TransPointCloud
from sensor_msgs.msg import Image, Range, PointCloud2

import time
import ros_numpy

class ScaleRecovery:
    def __init__(self):
        self._bridge = CvBridge()
        self._w = 640
        self._h = 480
        self._SCALE_FACTOR = 0
        self._RELATIVE_DEPTH = 0
        self._METRIC_DEPTH = 0
        self._cam_link = 'camera_link'
        # -0.03 translation between camera_link and base_link, while ignored.
        self._sensor_incidence = 10
        self._trans = [0, 0, 0]
        self._euler = [-math.pi*(90 + self._sensor_incidence)/180, 0, -math.pi*90/180]
        self._SCALE_FACTOR_FILTER = np.array([])
        self._filter_shape = 10
        self._trans_mat = TransPointCloud().transform_matrix(self._trans, self._euler)

        # ROS publishers
        self._depth_pub = rospy.Publisher('/sync/depth_metric', Image, queue_size=10)
        
        # ROS subscribers
        self._point_sub = message_filters.Subscriber('/sync/pcl_ground', PointCloud2)
        self._height_sub = message_filters.Subscriber('/tello/tof_btm', Range)
        self._data_pack = message_filters.ApproximateTimeSynchronizer([self._point_sub, self._height_sub], 10, 0.1, allow_headerless=True)
        self._data_pack.registerCallback(self.scale_factor_callback)
        rospy.Subscriber('/sync/depth', Image, self.metric_depth_callback)


    # Calculate height between body & ground
    def relative_height_estimate(self, point_msg_ground):
        # np.array will be returned
        ground_pcl_base = TransPointCloud().trans_pcd_ground(self._trans_mat, point_msg_ground, 'base_link', 0)

        # ground pcl Median filter!!!

        # height value
        ground = ground_pcl_base[2, :]
        relative_height = np.mean(ground)
        
        return relative_height


    def scale_factor_callback(self, point_msg, height_msg):
        # Relative Height Estimation
        relative_height = self.relative_height_estimate(point_msg)
        
        # Scale Factor
        metric_height = height_msg.range
        scale_factor =  np.absolute(metric_height/relative_height)
        # self._SCALE_FACTOR_FILTER = np.append(self._SCALE_FACTOR_FILTER, scale_factor)
        
        # Median Filter
        if len(self._SCALE_FACTOR_FILTER) < self._filter_shape:
            self._SCALE_FACTOR_FILTER = np.append(self._SCALE_FACTOR_FILTER, scale_factor)
            print('Initializing')
        
        # To avoid scale factor changes dramatically
        else:
            err = np.absolute(scale_factor - np.mean(self._SCALE_FACTOR_FILTER))
            if err < 0.5:
                self._SCALE_FACTOR_FILTER = np.append(self._SCALE_FACTOR_FILTER, scale_factor)
                # Delete the earliest element in the Scale Factor queue
                self._SCALE_FACTOR_FILTER = np.delete(self._SCALE_FACTOR_FILTER, 0)
            else:
                print('Scale Factor changes Dramatically!')


    def metric_depth_callback(self, depth_msg):
        if len(self._SCALE_FACTOR_FILTER) == self._filter_shape:
            # Relative Depth Map
            self._RELATIVE_DEPTH = self._bridge.imgmsg_to_cv2(depth_msg)
            
            # Scale Factor Median
            self._SCALE_FACTOR = np.median(self._SCALE_FACTOR_FILTER)
            print(self._SCALE_FACTOR)

            # Metric Depth Map
            # self._METRIC_DEPTH = self._RELATIVE_DEPTH * self._SCALE_FACTOR
            self._METRIC_DEPTH = self._RELATIVE_DEPTH * 5
            self._METRIC_DEPTH = self._bridge.cv2_to_imgmsg(self._METRIC_DEPTH, "32FC1")
            self._METRIC_DEPTH.header.stamp = depth_msg.header.stamp
            self._METRIC_DEPTH.header.frame_id = self._cam_link
            
            # Publish Metric Depth Map
            self._depth_pub.publish(self._METRIC_DEPTH)


    '''    
        stamp = depth_msg.header.stamp
        self.PlotDepthMap(stamp, self._RELATIVE_DEPTH)

    def PlotDepthMap(self, stamp, depthmap):
        test_depth = depthmap[:360,:480]
        test_depth = self._bridge.cv2_to_imgmsg(test_depth, "32FC1")
        test_depth.header.stamp = stamp
        test_depth.header.frame_id = 'camera_link'
        self._depth_pub_test.publish(test_depth)
    '''


if __name__ == '__main__':
    # Initialize ROS
    rospy.init_node('scale_recovery', anonymous=False)
    rospy.loginfo('Running until shutdown (Ctrl-C).')
    depth_inference_node = ScaleRecovery()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Node Shutdown")