#!/usr/bin/env python
# -*-coding:utf-8-*-

# ROS
import time
import rospy
import numpy as np
import ros_numpy as rosnp
import message_filters
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, PoseStamped
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from mav_controller.msg import Bspline

class State(object):
    """
    Class representing the state of a vehicle.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super(State, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

class Stanley:
    def __init__(self):
        self._k = 0.5  # control gain
        self._Kp = 1.0  # speed proportional gain
        self._dt = 0.1  # [s] time difference
        self._L = 0.2  # [m] Wheel base of vehicle
        self._max_steer = np.radians(30.0)  # [rad] max steering angle
        self._method_name = rospy.get_param('~algorithm_name')
        self._v = 0.25 
        self._max_w = 0.25
        self._factor_v = 1 
        self._factor_w = 1 
        self._state = State()

        self._path = np.array([])

        # ROS Publisher
        self._vel_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)
        
        # ROS Subscriber
        # Pose Subsciber
        rospy.Subscriber('/sync/tf_odom', Odometry, self.vel_cmd_callback, queue_size=10)
        
        # Path Subscriber
        if self._method_name == 'fast-planner':
            print('Fast-Planner Selected')
            rospy.Subscriber('/tracking_fsm_node/visualization/vis_smooth_traj', Marker, self.path_callback_marker, queue_size=10)
        
        if self._method_name == 'fast-tracker':
            print('Fast-Tracker Selected')
            rospy.Subscriber('/planning/bspline', Bspline, self.path_callback_bspline, queue_size=10)


    def path_callback_bspline(self, path_msg):
        path_points = path_msg.pos_pts
        num = len(path_points)

        if num == 0:
            pass
        else:
            print(num)
            path = np.empty(shape=[0, 3])        
            for i in range(num):
                path = np.append(path, [rosnp.numpify(path_points[i])], axis=0)
            self._path = path

    def path_callback_marker(self, path_msg):
        path_points = path_msg.points
        num = len(path_points)

        if num == 0:
            pass
        else:
            print(num)
            path = np.empty(shape=[0, 3])
            for i in range(num):
                path = np.append(path, [rosnp.numpify(path_points[i])], axis=0)
            self._path = path


    def stanley_control(self, state, cx, cy, cyaw, last_target_idx):
        """
        Stanley steering control.

        :param state: (State object)
        :param cx: ([float])
        :param cy: ([float])
        :param cyaw: ([float])
        :param last_target_idx: (int)
        :return: (float, int)
        """
        current_target_idx, error_front_axle = self.calc_target_index(state, cx, cy)
        # print(current_target_idx)
        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx

        # theta_e corrects the heading error
        theta_e = self.normalize_angle(cyaw[current_target_idx] - state.yaw)
        # theta_d corrects the cross track error
        theta_d = np.arctan2(self._k * error_front_axle, state.v)
        # Steering control
        delta = theta_e + theta_d
        # print(theta_e, theta_d)

        return delta, current_target_idx

    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].

        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    def calc_target_index(self, state, cx, cy):
        """
        Compute index in the trajectory list of the target.

        :param state: (State object)
        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        """
        # Calc front axle position
        fx = state.x + self._L * np.cos(state.yaw)
        fy = state.y + self._L * np.sin(state.yaw)

        # Search nearest point index
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                        -np.sin(state.yaw + np.pi / 2)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle

    def vel_cmd_callback(self, odom_msg):
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        z = odom_msg.pose.pose.position.z
        quaternion = [odom_msg.pose.pose.orientation.x, 
                      odom_msg.pose.pose.orientation.y, 
                      odom_msg.pose.pose.orientation.z, 
                      odom_msg.pose.pose.orientation.w]
        theta = euler_from_quaternion(quaternion)[2]

        if len(self._path) == 0:
            print('path empty')
        else:           
            # control quantity
            cx = self._path[:, 0]
            cy = self._path[:, 1]
            cz = self._path[:, 2]
            cyaw = np.arctan2(cy, cx)

            last_idx = len(cx) -1

            self._state.x = x
            self._state.y = y
            self._state.yaw = theta
            self._state.v = self._v

            target_idx, _ = self.calc_target_index(self._state, cx, cy)

            if last_idx > target_idx:
                di, target_idx = self.stanley_control(self._state, cx, cy, cyaw, target_idx)
                di = np.clip(di, -self._max_steer, self._max_steer)
                w = self._v / self._L * np.tan(di)
                w = w / np.pi * 180 / 100
                w = np.clip(w, -self._max_w, self._max_w)
                vel_msg = Twist()
                vel_msg.linear.x = self._v * self._factor_v
                vel_msg.angular.z = w * self._factor_w
            else:
                vel_msg = Twist()
        
            self._vel_pub.publish(vel_msg)

if __name__ == '__main__':
    # Initialize ROS
    rospy.init_node('path_following_controller', anonymous=False)
    rospy.loginfo('Running until shutdown (Ctrl-C).')
    info_pub_node = Stanley()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Node Shutdown")