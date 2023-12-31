#!/usr/bin/env python
# -*-coding:utf-8-*-

# ROS
import tf
import cv2
import yaml
import rospy
import tf2_ros
import numpy as np
import message_filters
from cv_bridge import CvBridge
from tf import transformations
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from scale_recovery.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped, PoseStamped

# Camera Info
def parse_yaml(file_name):
    with open(file_name, "r") as file_handle:
        calib_data = yaml.safe_load(file_handle)
    cam_info = CameraInfo()
    cam_info.width = calib_data['image_width']
    cam_info.height = calib_data['image_height']
    cam_info.K = calib_data['camera_matrix']['data']
    cam_info.D = calib_data['distortion_coefficients']['data']
    cam_info.R = calib_data['rectification_matrix']['data']
    cam_info.P = calib_data['projection_matrix']['data']
    cam_info.distortion_model = calib_data['distortion_model']
    return cam_info

# Static Transform
def StaticTrans(frame, child_frame, trans, euler):
    tfs = TransformStamped()
    tfs.header.stamp = rospy.Time.now()
    tfs.header.frame_id = frame
    tfs.child_frame_id = child_frame
    tfs.transform.translation.x = trans[0]
    tfs.transform.translation.y = trans[1]
    tfs.transform.translation.z = trans[2]
    qtn = transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
    tfs.transform.rotation.x = qtn[0]
    tfs.transform.rotation.y = qtn[1]
    tfs.transform.rotation.z = qtn[2]
    tfs.transform.rotation.w = qtn[3]
    return tfs

class InfoPub: 
    def __init__(self):
        self._fixed_scale = 5
        self._bridge = CvBridge()
        self. _file_name = rospy.get_param('~camera_info_path')
        self._CAM_INFO = parse_yaml(self._file_name)
        self._w = 640
        self._h = 480
        self._cam2body_trans = (0, 0, 0)
        self._cam2body_rot = transformations.quaternion_from_euler(np.deg2rad(-90), 0, np.deg2rad(-90))

        # ROS subscribers
        self._rgb_sub =  message_filters.Subscriber('/sc_depth_pl/color/image_raw', Image)
        self._depth_sub = message_filters.Subscriber('/sc_depth_pl/depth/image_raw', Image)
        self._data_pack = message_filters.ApproximateTimeSynchronizer([self._rgb_sub, self._depth_sub], 10, 0.1, allow_headerless=True)
        self._data_pack.registerCallback(self.InfoCallback)
        rospy.Subscriber('/orb_slam2_rgbd/pose', PoseStamped, self.tfOdomCallback)
        rospy.Subscriber('/orb_slam2_rgbd/pose', PoseStamped, self.tfCallback)
        rospy.Subscriber('/orb_slam2_rgbd/pose', PoseStamped, self.camPoseCallback)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tagOdomCallback)
        
        # ROS publishers
        self._info_pub = rospy.Publisher('/sync/camera_info', CameraInfo, queue_size=10)
        self._depth_pub = rospy.Publisher('/sync/depth_metric', Image, queue_size=10)
        self._image_pub = rospy.Publisher('/sync/image', Image, queue_size=10)
        self._pose_pub = rospy.Publisher('/sync/camera_pose', PoseStamped, queue_size=10)
        self._odom_tf_pub = rospy.Publisher('/sync/tf_odom', Odometry, queue_size=10)
        self._odom_tag_pub = rospy.Publisher('/sync/tag_odom', Odometry, queue_size=10)
        

        # ROS listener & broadcaster
        self._listener = tf.TransformListener()
        self._broadcaster = tf.TransformBroadcaster()
        self._map_frame = 'map'
        self._base_link_frame = 'base_link'
        self._camera_frame = 'camera_link'
    
    # Multi-info callback
    def InfoCallback(self, rgb_msg, depth_msg):
        # Depth
        idepth = cv2.resize(self._bridge.imgmsg_to_cv2(depth_msg, "32FC1"), (self._w, self._h))
        idepth = idepth * self._fixed_scale
        idepth = self._bridge.cv2_to_imgmsg(idepth, "32FC1")
        idepth.header.seq = depth_msg.header.seq
        idepth.header.stamp = depth_msg.header.stamp
        idepth.header.frame_id = 'camera_link'
        
        # Camera Info
        self._CAM_INFO.header.seq = depth_msg.header.seq
        self._CAM_INFO.header.stamp = depth_msg.header.stamp
        self._CAM_INFO.header.frame_id = 'camera_link'

        # RGB Image
        image = cv2.resize(self._bridge.imgmsg_to_cv2(rgb_msg), (self._w, self._h))     
        image = self._bridge.cv2_to_imgmsg(image, "bgr8")
        image.header.seq = depth_msg.header.seq
        image.header.stamp = depth_msg.header.stamp
        image.header.frame_id = 'camera_link'

        # Publish Info 
        self._image_pub.publish(image)
        self._depth_pub.publish(idepth) 
        self._info_pub.publish(self._CAM_INFO)

    # Static Transform between frame 'base_link' and 'camera_link'
    def tfCallback(self, pose_msg):
        self._broadcaster.sendTransform(self._cam2body_trans,
                                        self._cam2body_rot,
                                        rospy.Time.now(),
                                        self._camera_frame,
                                        self._base_link_frame)
        # self._broadcaster.sendTransform(self._cam2body_trans,
        #                                 self._cam2body_rot,
        #                                 pose_msg.header.stamp,
        #                                 self._camera_frame,
        #                                 self._base_link_frame)
    
    # Pose of 'camera_link' under frame 'map'
    def camPoseCallback(self, pose_msg):
        try:
            pose = PoseStamped()
            pose.header.stamp = pose_msg.header.stamp
            pose.header.frame_id = 'map'

            # Learn base_link pose from tf broadcast
            (trans, rot) = self._listener.lookupTransform('map', 'camera_link', rospy.Time(0))
            # (trans, rot) = self._listener.lookupTransform('map', 'camera_link', pose.header.stamp)

            # Body translation
            [pose.pose.position.x, 
            pose.pose.position.y, 
            pose.pose.position.z] = trans

            # Body rotation
            [pose.pose.orientation.x, 
             pose.pose.orientation.y,
             pose.pose.orientation.z,
             pose.pose.orientation.w] = rot 

            self._pose_pub.publish(pose)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
    
    # Odometry of 'body_link' under frame 'map'
    def tfOdomCallback(self, pose_msg):
        try:
            odom = Odometry()
            odom.header.stamp = pose_msg.header.stamp
            odom.header.frame_id = 'map'
            odom.child_frame_id = 'body_link'

            # Learn base_link pose from tf broadcast
            # (trans, rot) = self._listener.lookupTransform('map', 'base_link', rospy.Time(0))
            (trans, rot) = self._listener.lookupTransform('map', 'base_link', odom.header.stamp)

            # Body translation
            [odom.pose.pose.position.x, 
            odom.pose.pose.position.y, 
            odom.pose.pose.position.z] = trans

            # Body rotation
            [roll_body, pitch, yaw_body] = transformations.euler_from_quaternion(rot)
            pitch_body = pitch - np.deg2rad(10.0)
            rot_body = transformations.quaternion_from_euler(roll_body, pitch_body, yaw_body)
            [odom.pose.pose.orientation.x, 
             odom.pose.pose.orientation.y,
             odom.pose.pose.orientation.z,
             odom.pose.pose.orientation.w] = rot_body
      
            odom.pose.covariance = np.diag([1e-2, 1e-2, 1e-2, 1e3, 1e3, 1e-1]).ravel()

            self._odom_tf_pub.publish(odom)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
    
    # Odometry of tag under frame 'map'
    def tagOdomCallback(self, tag_msg):
        try:
            detection = bool(tag_msg.detections)
            if detection == True:
                odom = Odometry()
                odom.header.stamp = rospy.Time(0)
                odom.header.frame_id = 'map'
                odom.child_frame_id = 'tag_link'
            
                # Learn tag_link pose from tf broadcast
                # (trans, rot) = self._listener.lookupTransform('map', 'tag_5', rospy.Time(0))
                (trans, rot) = self._listener.lookupTransform('map', 'tag_5', odom.header.stamp)

                # tag translation, while ignoring tag rotation
                [odom.pose.pose.position.x, 
                 odom.pose.pose.position.y, 
                 odom.pose.pose.position.z] = trans

                odom.pose.covariance = np.diag([1e-2, 1e-2, 1e-2, 1e3, 1e3, 1e-1]).ravel()

                self._odom_tag_pub.publish(odom)
            else:
                pass
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass  


if __name__ == '__main__':
    # Initialize ROS
    rospy.init_node('info_proc', anonymous=False)
    rospy.loginfo('Running until shutdown (Ctrl-C).')
    
    # Static Transform between frame 'map' and 'world'
    trans = [0, 0, 0]
    euler = [0, np.deg2rad(10.0), 0]
    tf_pub = tf2_ros.StaticTransformBroadcaster()
    STATIC_TRANS = StaticTrans('map', 'world', trans, euler)
    tf_pub.sendTransform(STATIC_TRANS)

    # Multi-Info Processing 
    info_pub_node = InfoPub()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Node Shutdown")