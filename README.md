
# PhaRaO with GTSAM

This repo reproduces the code for the paper [PhaRaO: Direct Radar Odometry using Phase Correlation.](https://rpm.snu.ac.kr/publications/yspark-2020-icra.pdf)

In the paper, the original PhaRaO is based on ISAM, however, we adapted the code using ISAM2 in GTSAM


## Prerequisite
1. [ROS](https://wiki.ros.org/noetic/Installation/Ubuntu)
2. [GTSAM](https://gtsam.org/)
3. [FFTW3](https://www.fftw.org/download.html)
   (fftw-3.3.10 version is recommended for ubuntu 20.04)

   If **_FFTW3LibraryDepend.cmake_** file related error occurred, don't use given configure file, build with cmake.

4. Download Radar Data samples (ex. [MulRan](https://sites.google.com/view/mulran-pr/dataset))

## Start
1. Generate your own catkin workspace & Go to your src folder
   ~~~
   mkdir ~/catkin_ws
   cd ~/catkin_ws
   mkdir src
   cd src
   ~~~
2. Download source file for PhaRaO
   ~~~
   git clone https://github.com/rpmsnu/PhaRaO_gtsam.git
   ~~~

3. Compile the source file
   ~~~
   cd ~/catkin_ws
   catkin_make
   ~~~
4. Launch File Player Module(MulRan for example)
    ~~~
    roslaunch file_player file_player.launch
    ~~~
5. Run PhaRaO node (It will be alternated with launch file later)
    ~~~
    rosrun pharao_gtsam pharao_gtsam_node
    ~~~

## Parameter Changing
If your bag file fails generating odometry, revise the parameter in file **_radar_odom.cpp_**

Line 391: Cost for generating odometry factor

Line 559,564,591: Cost for keyframe factor
