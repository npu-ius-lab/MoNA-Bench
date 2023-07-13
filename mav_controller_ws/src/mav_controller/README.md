# mav_controller_ws

The repository is a clone from [rmtt_ros / rmtt_tracker](https://github.com/cavayangtao/rmtt_ros/tree/main/rmtt_tracker), while some modifications have been made to satisfy our requirement. Please follow this ```readme``` file to complete required configuration.

## Build on ROS
You can find ```mav_controller_ws``` under the repository ```Mono_Drone_Eva```, and please type the following commands to compile it.: 

```
cd ~/Mono_Drone_Eva/mav_controller_ws
catkin_make
```

## Run the Program
After compilation you can start the program. Two controller are included in this package: a PID controller and a path following one. Both the nodes are tested in Ubuntu 18.04/ROS Melodic. 

The PID controller is recommended using (and only can be used) in autonomous obstacle avoidance with [Fast-Planner](https://github.com/npu-ius-lab/Mono_Drone_Eva/tree/main/fast_tracker_ws/src/Fast-tracker). The path following controller is a appropriate choice for target tracking with [Fast-tracker](https://github.com/npu-ius-lab/Mono_Drone_Eva/tree/main/fast_tracker_ws/src/Fast-tracker).

For ```pid_controller```, please run :
```
  cd ~/Mono_Drone_Eva/mav_controller_ws
  source devel/setup.bash
  roslaunch mav_controller pid_controller.launch  
```

To run the path following controller, please type :

```
  cd ~/Mono_Drone_Eva/mav_controller_ws
  source devel/setup.bash
  roslaunch mav_controller stanley_controller.launch
```


##
# BELOW IS THE ORIGINAL  ```readme``` FILE 

rmtt_node.py is tested in Ubuntu 20.04/ROS Noetic.

rmtt_node18.py is tested in Ubuntu 18.04/ROS Melodic.

Please switch in rmtt_bringup.launch.

To run rmtt_node18.py:
```
conda create -n py38 python=3.8
conda activate py38
pip install robomaster==0.1.1.63
pip install rospkg
pip install pyyaml
pip install scipy
```
add "-i https://pypi.tuna.tsinghua.edu.cn/simple" when run pip install in China.

RoboMaster Tello Talent ROS Driver is developed initially for [NPU-RoboCourse](https://github.com/cavayangtao/npurobocourse) through the cooperation of [NPU-USRI](https://wurenxitong.nwpu.edu.cn/) and Tianbot.


# RoboMaster Tello Talent (RMTT)
RoboMaster TT (also know as DJI Tello Talent or Ryze Tello Talent) features improved hardware and an LED light. RoboMaster TT suports Wi-Fi 5G channel and a flying map, which can be used for low-cost drone swarm. 


## Specification
|  Items  | Details|
|    :---:    |     :---:     |
| Model  | TLW004 |
| Weight  | 87 g |
| Max Speed  | 28.8 kph |
| Max Flight Time  | 13 min |
| Camera  | 2592×1936 |
| Battery  | LiPo |
| Capacity  | 1100 mAh |
| Voltage  | 3.8 V |


## Installation Instructions
Only support authorized devices and the environment is pre-configured in ROS2GO Noetic version.

## Usage Instructions
Please read the RoboMaster TT User Manual carefully.
Switch RoboMaster TT to direct connection mode, connect to RMTT-XXXXXX network.

### Start rmtt_driver node

```
roslaunch rmtt_driver rmtt_bringup.launch
```

## Topics
Published topics:
  - ~imu_data [sensor_msgs/Imu] 
  imu data. 

  - ~pose [geometry_msgs/PoseStamped] 
  Drone pose on the mission pad.

  - ~mission_pad_id [std_msgs/UInt8] 
  Mission pad id. 1 - 12.

  - ~image_raw/compressed [sensor_msgs/CompressedImage] 
  Compressed image topic.

  - ~image_raw [sensor_msgs/Image] 
  Image topic.

  - ~camera_info [sensor_msgs/CameraInfo] 
  RMTT camera info.

  - ~tof_btm [sensor_msgs/Range] 
  Bottom tof. Distance from drone to the ground.

  - ~tof_ext [sensor_msgs/Range] 
  External tof. Distance to the obstacle in front of the drone.

  - ~altitude [std_msgs/Float32] 
  Barometer readings.

  - ~battery [std_msgs/Float32] 
  Battery percentage.

Subscribed topics:
  - ~cmd_vel [geometry_msgs/Twist] 
  Command velocity. Only be effected after taking off.

  - ~takeoff [std_msgs/Empty] 
  Takeoff.

  - ~land [std_msgs/Empty]
  Land.

  - ~flip [std_msgs/Empty] 
  Flip.

  - ~led [std_msgs/ColorRGBA] 
  Led on top of the external module.

  - ~mled [std_msgs/String] 
  Dot matrix display. Showing charactor or string.

Services:
  - ~set_downvision [std_srvs/SetBool]
  Change the front cam to downward cam

  - ~set_hdmap
  Change the positioning to HD map mode. Can be used with the mission pad or HD map.

## License
The ROS driver is prietary. Other packages are under BSD Clause-3 open source license. RoboMaster SDK is under Apache License, version 2.0.
