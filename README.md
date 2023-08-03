
# PhaRaO (w/ GTSAM)

This repo reproduces the code for the paper [PhaRaO: Direct Radar Odometry using Phase Correlation.](https://rpm.snu.ac.kr/publications/yspark-2020-icra.pdf)
Although the original paper is based on iSAM, this release integrate the code with gtSAM.


## Prerequisite
1. [ROS](https://wiki.ros.org/noetic/Installation/Ubuntu) The code has been tested using ROS Noetic? (TODO)
2. [GTSAM](https://gtsam.org/)
3. [FFTW3](https://www.fftw.org/download.html)
   (fftw-3.3.10 version is recommended for ubuntu 20.04)

   If **_FFTW3LibraryDepend.cmake_** file-related error occurred, try the below lines.

   $ cmake .  
   $ ./configure --enable-shared --enable-threads --enable-float  
   $ make  
   $ sudo make install  
   $ sudo cp FFTW3LibraryDepends.cmake /usr/local/lib/cmake/fftw3/  

4. Download Radar Data samples (ex. [MulRan](https://sites.google.com/view/mulran-pr/dataset))

## Start
1. Generate your own catkin workspace & Go to your src folder
   ~~~
   mkdir ~/catkin_ws
   cd ~/catkin_ws
   mkdir src
   cd src
   ~~~
2. Download the source file for PhaRaO
   ~~~
   git clone https://github.com/hyesu-jang/PhaRaO_gtsam.git
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
5. Run PhaRaO node (It will be alternated with the launch file later)
    ~~~
    rosrun pharao_gtsam pharao_gtsam_node
    ~~~

## Parameter Changing
If your bag file fails to generate odometry, revise the parameter in the file **_radar_odom.cpp_**

1. [Cost](https://github.com/hyesu-jang/PhaRaO_gtsam/blob/258a9e1e354d34ad936613117b53aabf090398fc/src/radar_odom.cpp#L491) for generating odometry factor

2. [Cost](https://github.com/hyesu-jang/PhaRaO_gtsam/blob/258a9e1e354d34ad936613117b53aabf090398fc/src/radar_odom.cpp#L623) for keyframe factor
