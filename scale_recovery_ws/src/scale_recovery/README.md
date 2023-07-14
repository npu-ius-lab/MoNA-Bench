# scale_recovery_ws

The repository is a package to calculate metric scale factor from estimated depth map and metric height obtained by ToF sensor. Please follow this ```readme``` file to complete required configuration.

## Prerequisites
The package is tested in Ubuntu 18.04/ROS Melodic. Some C++ libraries like ```PCL``` are required in the program. Please run the following commands to complete configuration:
<!-- ```
conda create -n py38 python=3.8
pip install robomaster==0.1.1.63
pip install rospkg
pip install pyyaml
pip install scipy
``` -->

<!-- ------------------------ FIXME!!!! ------------------------ -->
<!-- ------------------------ FIXME!!!! ------------------------ -->
<!-- ------------------------ FIXME!!!! ------------------------ -->

## Build on ROS

You can find ```scale_recovery_ws``` under the repository ```Mono_Drone_Eva```, and please type the following commands to compile it.: 

```
cd ~/Mono_Drone_Eva/scale_recovery_ws
catkin_make
```

## Run the Program

After compilation you can start the program. The package is designed for __Monocular Depth Estimation (MDE)__ and several functions (nodes) are provided here:
- __Evaluation of MDE algorithms performance.__
- __Autonomous obstacle avoidance.__
- __Target tracking with real-time scale factor calculation.__
- __Target tracking with fixed scale factor.__

### 1. Evaluation of MDE algorithms
To evaluate algorithms performance, please run
```
  cd ~/Mono_Drone_Eva/scale_recovery_ws
  source devel/setup.bash
  roslaunch scale_recovery scale_recovery_evaluation.launch 
```
<!-- ------------------------ FIXME!!!! ------------------------ -->
to obtain the estimated distances while the UAV locates in a fixed position. The obtained distances as well as calculated scale factors are recorded in the folder ```evaluation_data``` under ```~/scale_recovery_ws/src```. The experimental site is shown in our [paper](). <!-- FIX HERE -->
<!-- ------------------------ FIXME!!!! ------------------------ -->


### 2. Autonomous obstacle avoidance
For autonomous obstacle avoidance, please run :
```
  cd ~/Mono_Drone_Eva/scale_recovery_ws
  source devel/setup.bash
  roslaunch scale_recovery scale_recovery_planner.launch 
```
It should be noted that [Fast-Planner](https://github.com/npu-ius-lab/Mono_Drone_Eva/tree/main/fast_planner_ws/src/Fast-Planner) is required to work cooperatively with this function. Please launch [Fast-Planner](https://github.com/npu-ius-lab/Mono_Drone_Eva/tree/main/fast_planner_ws/src/Fast-Planner) firstly before activate this node.

### 3. Target tracking with real-time scale factor calculation
To track a [AprilTag](https://april.eecs.umich.edu/software/apriltag) QR code target, please run :
```
  cd ~/Mono_Drone_Eva/scale_recovery_ws
  source devel/setup.bash
  roslaunch scale_recovery scale_recovery_tracker.launch 
```
It should be noted that [Fast-tracker](https://github.com/npu-ius-lab/Mono_Drone_Eva/tree/main/fast_tracker_ws/src/Fast-tracker) is required to work cooperatively with this function. Please launch [Fast-tracker](https://github.com/npu-ius-lab/Mono_Drone_Eva/tree/main/fast_tracker_ws/src/Fast-tracker) firstly before activate this node.

### 4. Target tracking with fixed scale factor
According to our experimental result and theoretical analysis, our system can also work well with fixed scale factor when the MDE algorithms are designed with scale-consistency. Hence, we provide a function to track target with fixed scale factor. Please refer to the following commands to run this function :
```
  cd ~/Mono_Drone_Eva/scale_recovery_ws
  source devel/setup.bash
  roslaunch scale_recovery scale_recovery_tracker_fixed.launch 
```
It should be noted that [Fast-tracker](https://github.com/npu-ius-lab/Mono_Drone_Eva/tree/main/fast_tracker_ws/src/Fast-tracker) is also required to work cooperatively with this function. Please launch [Fast-tracker](https://github.com/npu-ius-lab/Mono_Drone_Eva/tree/main/fast_tracker_ws/src/Fast-tracker) firstly before activate this node.

<!-- For ```rmtt_driver```, please run
```
  cd ~/Mono_Drone_Eva/rmtt_driver_ws
  conda activate py38
  source devel_isolated/setup.bash
  roslaunch rmtt_driver rmtt_bringup.launch 
```
to activate the tello driver. A drone-view image window will then appear.

For ```rmtt_teleop```, if you want to get the drone off the ground and control it via keyboards, please open a new terminal and type the following commands:

```
  cd ~/Mono_Drone_Eva/rmtt_driver_ws
  conda activate py38
  source devel_isolated/setup.bash
  roslaunch rmtt_teleop rmtt_teleop_key.launch
```
or you can control the drone via joys:

```
  cd ~/Mono_Drone_Eva/rmtt_driver_ws
  conda activate py38
  source devel_isolated/setup.bash
  roslaunch rmtt_teleop rmtt_teleop_joy.launch 
``` -->
