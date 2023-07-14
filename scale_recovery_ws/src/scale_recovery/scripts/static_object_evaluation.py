#!/usr/bin/env python
# -*-coding:utf-8-*-

# ROS
import rospy
import numpy as np
import pandas as pd
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class StaticEvaluation():
    def __init__(self):
        self._bridge = CvBridge()
        self._record = rospy.get_param('~data_recording')
        print(self._record)
        if self._record == True:
            self._depth_filter_2 = self._depth_filter_3 = self._depth_filter_4 = self._depth_filter_6 = np.array([])
            self._filter_max = rospy.get_param('~data_size')
            print(self._filter_max)
            self._method_name = rospy.get_param('~algorithm_name')
            self._path = str(rospy.get_param('~evaluation_path')) + '/' + str(self._method_name) + '_static_object_evaluation.csv'
            rospy.Subscriber('/sync/depth_metric', Image, self.static_evaluation_callback)
        else:
            rospy.Subscriber('/sync/depth_metric', Image, self.static_evaluation_datafree_callback)

        
        self._target_depth_pub = rospy.Publisher('/sync/depth_box', Image, queue_size=10)

    def static_evaluation_callback(self, depth_msg):
        depth = self._bridge.imgmsg_to_cv2(depth_msg)
        depth_roi = np.full(shape=depth.shape, fill_value=255, dtype="float32")

        # box 2
        depth_roi[80:430, 0:110] = depth_2 = depth[80:430, 0:110]
        distance_2 = np.mean(depth_2)
        self._depth_filter_2 = np.append(self._depth_filter_2, distance_2)

        # box 3
        depth_roi[130:350, 440:550] = depth_3 = depth[130:350, 440:550]
        distance_3 = np.mean(depth_3)
        self._depth_filter_3 = np.append(self._depth_filter_3, distance_3)

        # box 4.5
        depth_roi[140:260, 190:260] = depth_4 = depth[140:260, 190:260]
        distance_4 = np.mean(depth_4)
        self._depth_filter_4 = np.append(self._depth_filter_4, distance_4)
   
        # box 6
        depth_roi[140:230, 320:370] = depth_6 = depth[140:230, 320:370]
        distance_6 = np.mean(depth_6)
        self._depth_filter_6 = np.append(self._depth_filter_6, distance_6)
        
        depth_roi = self._bridge.cv2_to_imgmsg(depth_roi, "32FC1")
        depth_roi.header.seq = depth_msg.header.seq
        depth_roi.header.stamp = depth_msg.header.stamp
        depth_roi.header.frame_id = 'camera_link'
        self._target_depth_pub.publish(depth_roi)

        if len(self._depth_filter_6) == self._filter_max:
            column = ['2m', '3m', '4.5m', '6m']
            data = pd.DataFrame(np.array([
                                self._depth_filter_2, 
                                self._depth_filter_3, 
                                self._depth_filter_4, 
                                self._depth_filter_6
                                ]).T,
                                columns=column
                                )
            data.to_csv(self._path)
            print(self._path)
            print('Predicted Depth of Static Objects Evaluation Completed.')

    def static_evaluation_datafree_callback(self, depth_msg):
        depth = self._bridge.imgmsg_to_cv2(depth_msg)
        depth_roi = np.full(shape=depth.shape, fill_value=255, dtype="float32")

        # box 2
        depth_roi[80:430, 0:110] = depth_2 = depth[80:430, 0:110]
        distance_2 = np.mean(depth_2)

        # box 3
        depth_roi[130:350, 440:550] = depth_3 = depth[130:350, 440:550]
        distance_3 = np.mean(depth_3)

        # box 4.5
        depth_roi[140:260, 190:260] = depth_4 = depth[140:260, 190:260]
        distance_4 = np.mean(depth_4)
   
        # box 6
        depth_roi[140:230, 320:370] = depth_6 = depth[140:230, 320:370]
        distance_6 = np.mean(depth_6)

        print('Box 2:'+ str(distance_2) + 'm')
        print('Box 3:' + str(distance_3) + 'm')
        print('Box 4.5:'+ str(distance_4) + 'm')
        print('Box 6:' + str(distance_6) + 'm')

        depth_roi = self._bridge.cv2_to_imgmsg(depth_roi, "32FC1")
        depth_roi.header.seq = depth_msg.header.seq
        depth_roi.header.stamp = depth_msg.header.stamp
        depth_roi.header.frame_id = 'camera_link'
        self._target_depth_pub.publish(depth_roi)


if __name__ == '__main__':
    # Initialize ROS
    rospy.init_node('info_proc', anonymous=False)
    rospy.loginfo('Running until shutdown (Ctrl-C).')

    # Multi-Info Processing 
    info_pub_node = StaticEvaluation()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Node Shutdown")