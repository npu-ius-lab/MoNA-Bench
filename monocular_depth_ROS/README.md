# ROS Packages for Monocular Depth Estimation
The folder is a collection of 4 ROS packages for monocular depth estimation, includes [MonoDepth](https://github.com/tentone/monodepth), [MiDaS](https://github.com/isl-org/MiDaS), [PackNet_SfM](https://github.com/surfii3z/packnet_sfm_ros) and [SC_DepthV2](https://github.com/YongzhouPan/sc_depth_pl_ros).

 <!-- You can click the specific package to access its homepage. The performance of these algorithms was evaluated in our paper, and we are glad to show our great respect to the packages developers for their contribution to our open-source community. -->

**It should be noticed that the 4 packages were modified to satisfy our experimental requirements.** For more details, please check [monodepth_ros](https://github.com/YongzhouPan/monodepth_ros), [MiDaS_ros](https://github.com/YongzhouPan/MiDaS_ros), [packnet_sfm_ros](https://github.com/YongzhouPan/packnet_sfm_ros), [SC_Depth_pl_ros](https://github.com/YongzhouPan/sc_depth_pl_ros), respectively. 

<!-- ## Experimental Configuration
We have tested all the packages on `Ubuntu 18.04/ROS Melodic`. -->

## An Example with SC_Depth_pl_ros

Install in sc_depth_pl_ws/src/sc_depth_pl_ros/src:

'''
conda create -n sc_depth_env python=3.6
conda activate sc_depth_env
conda install pytorch==1.10.1 torchvision==0.11.2 cudatoolkit=11.3 -c pytorch -c conda-forge
pip install -r requirements.txt
'''

To config the workspace to work with Python3, in the workspace of sc_depth_pl_ws:

'''
sudo apt-get install python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg
sudo apt-get install python-catkin-tools python3-dev python3-numpy
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin config --install
catkin build
'''

Put files in sc_depth_pl_ws/src/sc_depth_pl_ros into sc_depth_pl_ws/install/share/sc_depth_pl_ros.

Put [nyu_scv2_model](https://onedrive.live.com/?id=36712431A95E7A25!3266&resid=36712431A95E7A25!3266&authkey=!AN9KaLjLL78kdKY&cid=36712431a95e7a25) file into sc_depth_pl_ws/install/share/sc_depth_pl_ros/src/ckpts, and run:

'''
source install/setup.bash --extend
roslaunch sc_depth_pl_ros sc_depth_ros.launch
'''