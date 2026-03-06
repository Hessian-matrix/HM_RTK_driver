cmake_minimum_required(VERSION 3.5)
project(hm_rtk)

set(CMAKE_BUILD_TYPE "Release") # Release, Debug or RelWithDebInfo
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
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

# 第三方库配置 - 使用相对路径，支持工作空间移动
set(HM_RTK_THIRDPARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty")
set(HM_RTK_CERES_DIR "${HM_RTK_THIRDPARTY_DIR}/ceres_solver")

# Eigen3：优先使用本地拷贝（可移动）
set(Eigen3_DIR "${HM_RTK_CERES_DIR}/share/eigen3/cmake" CACHE PATH "Eigen3 CMake dir" FORCE)
find_package(Eigen3 REQUIRED NO_MODULE)

# Ceres 依赖：尽量自动定位 glog/gflags 的 CMake 配置
set(_HM_RTK_GLOG_DIR_CANDIDATES
    "${HM_RTK_THIRDPARTY_DIR}/glog/lib/cmake/glog"
    "${HM_RTK_THIRDPARTY_DIR}/glog/lib64/cmake/glog"
    "/usr/local/lib/cmake/glog"
    "/usr/local/lib64/cmake/glog"
    "/usr/lib/cmake/glog"
    "/usr/lib64/cmake/glog"
)
if(CMAKE_LIBRARY_ARCHITECTURE)
    list(APPEND _HM_RTK_GLOG_DIR_CANDIDATES
        "/usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/glog"
        "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/glog")
endif()
if(NOT glog_DIR)
    foreach(_dir IN LISTS _HM_RTK_GLOG_DIR_CANDIDATES)
        if(EXISTS "${_dir}/glog-config.cmake" OR EXISTS "${_dir}/glogConfig.cmake")
            set(glog_DIR "${_dir}" CACHE PATH "glog CMake dir" FORCE)
            break()
        endif()
    endforeach()
endif()

set(_HM_RTK_GFLAGS_DIR_CANDIDATES
    "${HM_RTK_THIRDPARTY_DIR}/gflags/lib/cmake/gflags"
    "${HM_RTK_THIRDPARTY_DIR}/gflags/lib64/cmake/gflags"
    "/usr/local/lib/cmake/gflags"
    "/usr/local/lib64/cmake/gflags"
    "/usr/lib/cmake/gflags"
    "/usr/lib64/cmake/gflags"
)
if(CMAKE_LIBRARY_ARCHITECTURE)
    list(APPEND _HM_RTK_GFLAGS_DIR_CANDIDATES
        "/usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/gflags"
        "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/gflags")
endif()
if(NOT gflags_DIR)
    foreach(_dir IN LISTS _HM_RTK_GFLAGS_DIR_CANDIDATES)
        if(EXISTS "${_dir}/gflags-config.cmake" OR EXISTS "${_dir}/gflagsConfig.cmake")
            set(gflags_DIR "${_dir}" CACHE PATH "gflags CMake dir" FORCE)
            break()
        endif()
    endforeach()
endif()

# Ceres：复用本地 CMake 配置，自动带出依赖
set(Ceres_DIR "${HM_RTK_CERES_DIR}/lib/cmake/Ceres" CACHE PATH "Ceres CMake dir" FORCE)
find_package(Ceres REQUIRED CONFIG)

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
    target_link_libraries(calib_rtk_slam_node
        ${ROS_LIBRARIES}
        ${CERES_LIBRARIES})
else()
    ament_target_dependencies(calib_rtk_slam_node ${ROS_DEPENDENCIES})
    target_link_libraries(calib_rtk_slam_node
        ${CERES_LIBRARIES})
endif()

# ROS2特有配置
if(ROS_VERSION EQUAL 2)
    install(TARGETS rtk_driver_node calib_rtk_slam_node
        DESTINATION lib/${PROJECT_NAME})
    
    install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)
        
    ament_package()
endif()
