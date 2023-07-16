# mav_controller_ws

The repository refers to [rmtt_ros / rmtt_tracker](https://github.com/cavayangtao/rmtt_ros/tree/main/rmtt_tracker), while some modifications have been made to satisfy our requirement. Please follow this ```readme``` file to complete required configuration.

## Build on ROS
You can find ```mav_controller_ws``` under the repository ```Mono_Drone_Eva```, and please type the following commands to compile it.: 

```
cd ~/Mono_Drone_Eva/mav_controller_ws
catkin_make
```

## Run the Program
After compilation you can start the program. Two controller are included in this package: a PID controller and a path following one. Both the nodes are tested in Ubuntu 18.04/ROS Melodic. 

The PID controller is recommended to use (and only can be used) in autonomous obstacle avoidance with [Fast-Planner](https://github.com/npu-ius-lab/Mono_Drone_Eva/tree/main/fast_tracker_ws/src/Fast-tracker). The path following controller is a appropriate choice for target tracking with [Fast-tracker](https://github.com/npu-ius-lab/Mono_Drone_Eva/tree/main/fast_tracker_ws/src/Fast-tracker).

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
