# ME5413 Autonomous Mobile Robot #
Task details are in the file 'ME5413_HW2.pdf'
## Homework 2 slam  ##

### 1. Setup ###

#### 1.1. Install Python 3.9 and ros noetic ####

you can install python 3.9 and ros noetic by following the instructions in the following links:

- [Python 3.9](https://linuxize.com/post/how-to-install-python-3-9-on-ubuntu-20-04/)
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [ROS Noetic Vision Msgs](http://wiki.ros.org/vision_msgs)

Here we would like to thank fishros for providing the following command to install ros noetic:

```bash
wget http://fishros.com/install -O fishros && . fishros 
````

#### 1.2. Install Required Packages and library ####

- [Cartographer](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html)
- [A-Loam](https://github.com/HKUST-Aerial-Robotics/A-LOAM)
- [FAST_LIO](https://github.com/hku-mars/FAST_LIO.git)

Note: you need to deactivate your conda environment before you cmake the cartographer and you can also use the command
provided by fishros to easily install the Cartographer. You can use the following command to deactivate the conda
enviorment:
#### 1.3. data Perparing ####
please put the bag in  following folder:
- 2dlidar.bag HW2_SLAM/Task_1
- have_fun.bag HW2_SLAM/Task 2/ and catkin_ws/src/hw_2_pkg/data
- have_fun_tum.bag HW2_SLAM/Task 2/ and catkin_ws/src/hw_2_pkg/data
- have_fun_kitti.bag HW2_SLAM/Task 2/
- 

```bash
conda deactivate
```

### 2. Task 1 2D SLAM ###

#### 2.1 run cartographer ####
Before run the code please open the rosmaster by using the following command:
```bash
roscore
```
And then run the following command to reord the data:
```bash
cd src/hw_2_pkg/scripts
rosbag record -O catkin_ws/src/hw_2_pkg/scripts/gt.bag /tf /tf_static
```

```bash
cd catkin_ws/src/hw_2_pkg/scripts
python3 task_1.py
```
And then run the following command to run the cartographer:
```bash
conda deactivate
conda deactivate
cd cartographer_ws && source install_isolated/setup.bash
cd ../HW2_SLAM/Task_1/
roslaunch cartographer_ros demo_revo_lds.launch bag_filename:=${HOME}/ME5413/HW2_SLAM/Task_1/2dlidar.bag
```
After the rosbag finish, you can run the following command to read the groundtruth:
```bash
cd catkin_ws/src/hw_2_pkg/scripts
python3 a.py gt.bag groundtruth.tum
```
#### 2.1 create map ####
```bash
cd cartographer_ws && source install_isolated/setup.bash
rosservice call /finish_trajectory 0
rosservice call /write_state "{filename: '${HOME}/ME5413/HW2_SLAM/Task_1/bag.pbstream'}"
rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename ${HOME}/ME5413/HW2_SLAM/Task_1/bag.pbstream -map_filestem ${HOME}/ME5413/HW2_SLAM/Task_1/bag -resolution 0.05
```
#### 2.2 ####
You can use the following command to run the evo test:
```bash
cd catkin_ws/src/hw_2_pkg/scripts
evo_ape tum groundtruth.tum carto.tum --plot --plot_mode xy -va
```
### 3. Task 2 3D SLAM ###
#### 3.1 A-LOAM ####
#### 3.1.1 Introduction of A-LOAM ####
A-LOAM, or Advanced LiDAR Odometry and Mapping, is an enhanced version of the LOAM (LiDAR Odometry and Mapping) algorithm, which is widely recognized in the field of robotics and autonomous navigation for its precision and efficiency in 3D mapping and real-time localization using LiDAR (Light Detection and Ranging) data. A-LOAM optimizes the original LOAM framework to offer improved stability, accuracy, and computational efficiency, making it highly suitable for various applications, including autonomous driving, UAVs (Unmanned Aerial Vehicles), and robotic navigation in complex environments.

The essence of A-LOAM lies in its ability to separate the point cloud data processing into two main components: odometry and mapping. The odometry component estimates the robot's current motion based on the incoming LiDAR scans by detecting feature points in the environment, while the mapping component refines these estimates by creating a high-fidelity 3D map of the surroundings. This dual-stage process allows A-LOAM to maintain a balance between computational load and the quality of localization and mapping.

Significantly, A-LOAM introduces optimizations in feature extraction, selection algorithms, and the integration of robust optimization techniques to enhance the system's performance under diverse and challenging conditions. As a result, A-LOAM stands out as a robust and reliable solution for real-time SLAM applications, offering high precision and efficiency in processing LiDAR data for navigation and mapping tasks.

#### 3.2.2 Run the code ####
First create a txt file to save tum.txt
```bash
cd ~
mkdir txt
touch aloam_tum.txt
```
You can run the code with the following command:
```bash
cd ALOAM && source devel/setup.bash
roslaunch aloam_velodyne aloam_velodyne_HDL_32.launch
```
tum.txt will be saved in the txt file

## 3.1.3 run evo test ##
For evoluation, you can use the following command to run the evo test:
```bash
cd txt
evo_ape tum data/have_fun_tum.txt aloam_tum.txt -r full --plot --plot_mode xz
```
the result is shown like the following:
#### 3.2 FAST_LIO ####
#### 3.2.1 Introduction of FAST_LIO ####
Fast-LIO, standing for Fast Lidar-Inertial Odometry, is a state-of-the-art SLAM (Simultaneous Localization and Mapping) framework designed for real-time localization and mapping using LiDAR (Light Detection and Ranging) and IMU (Inertial Measurement Unit) data. Developed to address the challenges of high computational demand and latency in conventional LiDAR-Inertial systems, Fast-LIO offers a highly efficient and accurate solution suitable for various applications, including autonomous vehicles, robotics, and UAVs (Unmanned Aerial Vehicles).

At the core of Fast-LIO is its ability to rapidly process large volumes of 3D LiDAR point clouds and high-frequency IMU data. It employs an innovative odometry estimation technique that integrates LiDAR and IMU inputs seamlessly, leveraging advanced algorithms for point cloud registration and motion estimation. This integration not only enhances the system's robustness to diverse environments and dynamic conditions but also significantly reduces the computational resources required for real-time operation.

With its impressive speed and accuracy, Fast-LIO has become a popular choice for researchers and developers in the field of robotics and autonomous systems, facilitating a wide range of applications that require reliable real-time navigation and high-precision mapping.
#### 3.2.2 Run the code ####
As Fast-lio is about to a
You can run the code with the following command:
```bash
conda deactivate
conda deactivate
cd ws_livox && source devel/setup.bash
cd ../fast_lio_ws && source devel/setup.bash
cd ../catkin_ws && source devel/setup.bash
roslaunch hw_2_pkg hw_2.launch
```
And find the result in the following folder. 
- tum.txt: catkin_ws/src/hw_2_pkg/result/tum
- odometry.txt: catkin_ws/src/hw_2_pkg/result/bag
- scans.pcd: fast_lio_ws/src/FAST_LIO/PCD/scans.pcd

## 3.2.3 run evo test ##
For evoluation, you can use the following command to run the evo test:
```bash
cd catkin_ws/src/hw_2_pkg
evo_ape tum data/have_fun_tum.txt result/tum/output.txt -r full --plot --plot_mode xz
```