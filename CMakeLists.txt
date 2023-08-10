cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "-std=c++14")
project(pharao_gtsam)

# CMAKE TWEAKS
#========================================================================
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE RelWithDebInfo)
endif ()

# INCLUDE LOCAL MODULES
#========================================================================
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "./cmake")
file (GLOB MODULES "${CMAKE_MODULE_PATH}/*.cmake")
foreach (MODULE ${MODULES})
  include (${MODULE})
endforeach ()

set (SRC
  src/PhaRaO.cpp
  
  src/factor/Factor.cpp
  src/factor/OdomFactor.cpp
  src/factor/KeyFramingFactor.cpp

  src/imgProcess/ImageTF.cpp
  src/imgProcess/fftModule.cpp
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  tf
  cv_bridge
  eigen_conversions
  image_transport
  sensor_msgs
  pcl_conversions
  pcl_ros
)

find_package(Boost 1.65.1 COMPONENTS timer)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(FFTW3 REQUIRED)

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

find_package(GTSAMCMakeTools)
#include(GtsamMakeConfigFile)
#include(GtsamBuildTypes)
#include(GtsamTesting)


generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package (
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS roscpp std_msgs
  CATKIN_DEPENDS cv_bridge eigen_conversions image_transport roscpp rospy std_msgs sensor_msgs pcl_ros pcl_conversions
  DEPENDS Boost
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  ${SRC}
)

target_link_libraries (${PROJECT_NAME}
  gtsam
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  -lfftw3 -lfftw3f
  -lfftw3f_threads
)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node 
  ${PROJECT_NAME}
  gtsam
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  -lfftw3 -lfftw3f
  -lfftw3f_threads
)
