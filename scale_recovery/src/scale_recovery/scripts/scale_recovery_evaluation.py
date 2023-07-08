#!/usr/bin/env python
# -*-coding:utf-8-*-

# ROS
import cv2
import math
import rospy
import numpy as np
import pandas as pd
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
        # -0.03 translational displacement between camera_link and base_link, while ignored.
        self._trans = [0, 0, 0]
        self._euler = [-math.pi*100.0/180, 0, -math.pi*90/180]
        self._SCALE_FACTOR_FILTER = np.array([])
        self._trans_mat = TransPointCloud().transform_matrix(self._trans, self._euler)
        self._record = rospy.get_param('~data_recording')
        print(self._record)
        if self._record == True:
            self._filter_max = rospy.get_param('~data_size')
            print("data_size:", self._filter_max)
            self._method_name = rospy.get_param('~algorithm_name')
            self._path = str(rospy.get_param('~evaluation_path')) + '/' + str(self._method_name) + '_scale_recovery_evaluation.csv'
            self._evaluation = False

            self._point_sub = message_filters.Subscriber('/sync/pcl_ground', PointCloud2)
            self._height_sub = message_filters.Subscriber('/tello/tof_btm', Range)
            self._data_pack = message_filters.ApproximateTimeSynchronizer([self._point_sub, self._height_sub], 10, 0.5, allow_headerless=True)
            self._data_pack.registerCallback(self.scale_factor_callback)
            rospy.Subscriber('/sync/depth', Image, self.metric_depth_callback)
       
        else:
            self._point_sub = message_filters.Subscriber('/sync/pcl_ground', PointCloud2)
            self._height_sub = message_filters.Subscriber('/tello/tof_btm', Range)
            self._data_pack = message_filters.ApproximateTimeSynchronizer([self._point_sub, self._height_sub], 10, 0.5, allow_headerless=True)
            self._data_pack.registerCallback(self.scale_factor_callback)
            rospy.Subscriber('/sync/depth', Image, self.metric_depth_datafree_callback)

        # ROS topics
        

        # ROS publishers
        self._depth_pub = rospy.Publisher('/sync/depth_metric', Image, queue_size=10)
        
        # ROS subscribers
        

    # Calculate height between body & ground
    def relative_height_estimate(self, point_msg_ground):
        ground_pcl_base = TransPointCloud().trans_pcd_ground(self._trans_mat, point_msg_ground, 'base_link', 0)

        # height value
        ground = ground_pcl_base[2, :]
        relative_height = np.mean(ground)
        
        return relative_height


    def scale_factor_callback(self, point_msg, height_msg):
        # Relative Height Estimation
        relative_height = self.relative_height_estimate(point_msg)
        
        # Scale Factor
        metric_height = height_msg.range
        self._SCALE_FACTOR =  np.absolute(metric_height/relative_height)
        self._SCALE_FACTOR_FILTER = np.append(self._SCALE_FACTOR_FILTER, self._SCALE_FACTOR)
        
        # # Median Filter
        # if len(self._SCALE_FACTOR_FILTER) < self._filter_shape:
        #     self._SCALE_FACTOR_FILTER = np.append(self._SCALE_FACTOR_FILTER, scale_factor)
        #     print('Initializing')
        
        # # To avoid scale factor changes dramatically
        # else:
        #     err = np.absolute(scale_factor - np.mean(self._SCALE_FACTOR_FILTER))
        #     if err < 0.5:
        #         self._SCALE_FACTOR_FILTER = np.append(self._SCALE_FACTOR_FILTER, scale_factor)
        #         # Delete the earliest element in the Scale Factor queue
        #         self._SCALE_FACTOR_FILTER = np.delete(self._SCALE_FACTOR_FILTER, 0)
        #     else:
        #         print('Scale Factor changes Dramatically!')


    def metric_depth_callback(self, depth_msg):
        # Relative Depth Map
        self._RELATIVE_DEPTH = self._bridge.imgmsg_to_cv2(depth_msg)

        if self._SCALE_FACTOR != 0:
            # Metric Depth Map
            self._METRIC_DEPTH = self._RELATIVE_DEPTH * self._SCALE_FACTOR
            self._METRIC_DEPTH = self._bridge.cv2_to_imgmsg(self._METRIC_DEPTH, "32FC1")
            self._METRIC_DEPTH.header.stamp = depth_msg.header.stamp
            self._METRIC_DEPTH.header.frame_id = self._cam_link
            
            # Publish Metric Depth Map
            self._depth_pub.publish(self._METRIC_DEPTH)
            self._SCALE_FACTOR = 0
            # print('FACTOR USED!!!!!')

        if (len(self._SCALE_FACTOR_FILTER) == self._filter_max) & (self._evaluation == False):
            self._evaluation = True
            column = ['Calculated Scale Factor per frame']
            data = pd.DataFrame(np.reshape(self._SCALE_FACTOR_FILTER, (len(self._SCALE_FACTOR_FILTER), 1)), columns=column)
            data.to_csv(self._path)
            print('Scale Factor Evaluation Completed.')


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

    def metric_depth_datafree_callback(self, depth_msg):
        # Relative Depth Map
        self._RELATIVE_DEPTH = self._bridge.imgmsg_to_cv2(depth_msg)

        if self._SCALE_FACTOR != 0:
            # Metric Depth Map
            self._METRIC_DEPTH = self._RELATIVE_DEPTH * self._SCALE_FACTOR
            self._METRIC_DEPTH = self._bridge.cv2_to_imgmsg(self._METRIC_DEPTH, "32FC1")
            self._METRIC_DEPTH.header.stamp = depth_msg.header.stamp
            self._METRIC_DEPTH.header.frame_id = self._cam_link
            
            # Publish Metric Depth Map
            self._depth_pub.publish(self._METRIC_DEPTH)
            print('scale factor:' + str(self._SCALE_FACTOR))
            self._SCALE_FACTOR = 0
            # print('FACTOR USED!!!!!')


if __name__ == '__main__':
    # Initialize ROS
    rospy.init_node('scale_recovery', anonymous=False)
    rospy.loginfo('Running until shutdown (Ctrl-C).')
    depth_inference_node = ScaleRecovery()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Node Shutdown")