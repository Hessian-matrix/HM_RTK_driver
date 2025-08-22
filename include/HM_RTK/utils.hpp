#pragma once
#include <iostream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <list>
#include <boost/algorithm/string.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/asio.hpp>
#include <boost/regex.hpp>

#ifdef ROS1
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
using odometry_msg = nav_msgs::Odometry;
using NavSatFix_msg = sensor_msgs::NavSatFix;

#endif

#ifdef ROS2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
using odometry_msg = nav_msgs::msg::Odometry;
using NavSatFix_msg = sensor_msgs::msg::NavSatFix;
using string_msg = std_msgs::msg::String;
using poseStamped_msg = geometry_msgs::msg::PoseStamped;
#endif


#include <Eigen/Dense>
// Namespace Hessian 提供NMEA消息解析等实用函数
namespace Hessian {

    struct sixdof_pose_with_cov{
        int year;
        int mounth;
        int day;
        int hour;
        int second;
        int minute;
        int second_gps;/*GPS 周内毫秒 */
        int pose_type;
        int state;  /*Heading Status*/
        int sat;    //卫星数
        double pitch;
        double roll;
        double ecef_x;
        double ecef_y ;
        double ecef_z;
        double lat, lon,alt; /*经纬高*/
        double v_north,v_east,v_up; /*北方向速度 */
        Eigen::Quaterniond q;
        double xigema_ecef_x;
        double xigema_ecef_y;
        double xigema_ecef_z;
        double xigema_lat,xigema_lon,xigema_alt;
        double xigema_vx, xigema_vy, xigema_vz; /*北方向速度标准差 东方向速度标准差 天顶方向速度标准差*/
        int speed_type; /*0：速度解状态有效 1：速度解状态无效*/
    };
    /// 检查NMEA字符串的校验码
    bool checksum(const std::string &nmea);
    /// 解析发布的NMEA字符串为GNSS定位消息
    bool parse_pub_nmea(const std::string &nmea, NavSatFix_msg &gnss_pos_msg);
    /// 解析双天线rtk发布的6DOF AGRICA位置消息
    bool parse_pub_sixdof_pos(const std::string& unicore,sixdof_pose_with_cov& sixdof_pos_t);

    bool return_stence(std::string& buffer,std::string& stence);


}

