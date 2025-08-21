cmake_minimum_required(VERSION 3.5)
project(hm_rtk)

set(CMAKE_BUILD_TYPE "Release") # Release, Debug or RelWithDebInfo

# 检测ROS版本
if(DEFINED ENV{ROS_VERSION})
    set(ROS_VERSION $ENV{ROS_VERSION})
else()
    # 尝试查找ROS2
    find_package(rclcpp QUIET)
    if(rclcpp_FOUND)
        set(ROS_VERSION 2)
    else()
        # 默认ROS1
        set(ROS_VERSION 1)
    endif()
endif()

message(STATUS "Detected ROS Version: ${ROS_VERSION}")

# ROS1配置
if(ROS_VERSION EQUAL 1)
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
    )
    
    catkin_package()
    
    set(ROS_LIBRARIES ${catkin_LIBRARIES})
    set(ROS_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})
    add_definitions(-DROS1_BUILD)

# ROS2配置  
else()
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    
    set(ROS_DEPENDENCIES 
        rclcpp 
        std_msgs 
        sensor_msgs 
        geometry_msgs 
        nav_msgs
    )
    add_definitions(-DROS2_BUILD)
endif()

#ntrip
add_subdirectory(3rdParty/ntrip)

# 第三方库配置 - 使用更通用的查找方式
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLOG REQUIRED libglog)
set(Ceres_DIR "/home/ll/C/Libs/ceres-solver/ceres-solver-2.2.0/install/lib/cmake/Ceres/")
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS regex)

include_directories(
  include
  ${ROS_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  3rdParty/gnss_comm/include
  3rdParty/gnss_comm/include/gnss_comm
  3rdParty/Sophus
)
add_executable(rtk_driver_node
  src/rtk_driver_node.cpp 
  src/serial_hm.cpp
  src/utils.cpp
)

if(ROS_VERSION EQUAL 1)
    target_link_libraries(rtk_driver_node ${ROS_LIBRARIES} ntrip ${Boost_LIBRARIES})
else()
    ament_target_dependencies(rtk_driver_node ${ROS_DEPENDENCIES})
    target_link_libraries(rtk_driver_node ntrip ${Boost_LIBRARIES})
endif()

add_executable(calib_rtk_slam_node 
  src/calib_rtk_slam_node.cpp 
  src/trajectory_aligner.cpp
  3rdParty/gnss_comm/src/gnss_utility.cpp
)

if(ROS_VERSION EQUAL 1)
    target_link_libraries(calib_rtk_slam_node ${ROS_LIBRARIES} ${CERES_LIBRARIES} ${GLOG_LIBRARIES})
else()
    ament_target_dependencies(calib_rtk_slam_node ${ROS_DEPENDENCIES})
    target_link_libraries(calib_rtk_slam_node ${CERES_LIBRARIES} ${GLOG_LIBRARIES})
endif()

# ROS2特有配置
if(ROS_VERSION EQUAL 2)
    install(TARGETS rtk_driver_node calib_rtk_slam_node
        DESTINATION lib/${PROJECT_NAME})
    
    install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)
        
    ament_package()
endif()
