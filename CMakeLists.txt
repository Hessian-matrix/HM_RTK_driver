cmake_minimum_required(VERSION 3.0.2)
project(hm_rtk)

set(CMAKE_BUILD_TYPE "Release") # Release, Debug or RelWithDebInfo

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

#ntrip
add_subdirectory(3rdParty/ntrip)
add_subdirectory(3rdParty/gnss_comm)

find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES HM_RTK
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  3rdParty/gnss_comm/include
)
add_executable(rtk_driver_node
  src/rtk_driver_node.cpp 
  src/serial_hm.cpp
  src/utils.cpp
)
target_link_libraries(rtk_driver_node ${catkin_LIBRARIES} ntrip)

add_executable(calib_rtk_slam_node 
  src/calib_rtk_slam_node.cpp 
  src/trajectory_aligner.cpp
)
target_link_libraries(calib_rtk_slam_node ${catkin_LIBRARIES} ${CERES_LIBRARIES} gnss_comm glog::glog)
