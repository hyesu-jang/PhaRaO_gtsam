
# PhaRaO (w/ GTSAM)

This repo reproduces the code for the paper [PhaRaO: Direct Radar Odometry using Phase Correlation.](https://rpm.snu.ac.kr/publications/yspark-2020-icra.pdf)
Although the original paper is based on iSAM, this release integrate the code with gtSAM.

<p align="center"><img src="https://github.com/hyesu-jang/PhaRaO_gtsam/assets/30336462/655f5e0e-5391-423f-af94-4f94809e40b3" height=220> <img src="https://github.com/hyesu-jang/PhaRaO_gtsam/assets/30336462/28635bcd-02cc-4024-b8b7-f494cc4a9eb6" height=220></p>



## Prerequisite
1. [ROS](https://wiki.ros.org/noetic/Installation/Ubuntu) The code has been tested using ROS Noetic
2. [GTSAM](https://gtsam.org/)
   Use develop branch.
   
4. [FFTW3](https://www.fftw.org/download.html)
   (fftw-3.3.10 version is recommended for ubuntu 20.04)

   We need normal installation followed by float option installed. We need both `FFTW3Config.cmake` file and `FFTW3fConfig.cmake`. So first compile without float option.
   
   $ cmake .  
   $ ./configure --enable-shared --enable-threads --enable-float  
   $ make  
   $ sudo make install

   Then, do the same thing with the float option on.
   
   $ cmake .  
   $ ./configure --enable-shared --enable-threads --enable-float  
   $ make  
   $ sudo make install 
   
   If **_FFTW3LibraryDepend.cmake_** file-related error occurred, try the below lines. Check [this issue](https://github.com/hyesu-jang/PhaRaO_gtsam/issues/2)
 
   $ sudo cp FFTW3LibraryDepends.cmake /usr/local/lib/cmake/fftw3/  

6. Download Radar Data samples (ex. [MulRan](https://sites.google.com/view/mulran-pr/dataset))

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
5. Run PhaRaO node and rviz
    ~~~
    rosrun pharao_gtsam pharao_gtsam_node
    rviz -d ~/catkin_ws/src/PhaRaO_gtsam/rviz/odom.rviz
    ~~~
&nbsp; 5-1. Or run by using launch file
   ~~~
   roslaunch pharao_gtsam radar_odom.launch
   ~~~

&nbsp; 5-2. Saving result trajectory

   Enable save flag "True"
   `<param name="save_results_flag" value="false" />`

## Parameter Changing
If your bag file fails to generate odometry, revise the parameter in the file **_radar_odom.cpp_**

1. [Cost](https://github.com/hyesu-jang/PhaRaO_gtsam/blob/258a9e1e354d34ad936613117b53aabf090398fc/src/radar_odom.cpp#L491) for generating odometry factor

2. [Cost](https://github.com/hyesu-jang/PhaRaO_gtsam/blob/258a9e1e354d34ad936613117b53aabf090398fc/src/radar_odom.cpp#L623) for keyframe factor

## Maintainer

Hyesu Jang (dortz at snu dot ac dot kr)
Yeong Sang Park (pys0728k at gmail dot com)
