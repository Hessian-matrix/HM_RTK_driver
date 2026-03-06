#pragma once
#include <iostream>
#include <string>
#include "ros_adapter.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/asio.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>

// Namespace Hessian 提供NMEA消息解析等实用函数
namespace Hessian {

    struct sixdof_pose_with_cov{
        int year;
        int month;
        int day;
        int hour;
        int second;
        int minute;
        int second_gps;/*GPS 周内毫秒 */
        int pose_type;
        int heading_state;  /*Heading Status*/
        int speed_type; /*0：速度解状态有效 1：速度解状态无效*/
        int sat;    //卫星数
        double pitch;
        double roll;
        double ecef_x;
        double ecef_y ;
        double ecef_z;
        double lat, lon,alt; /*经纬高*/
        double v_north,v_east,v_up; /*北方向速度 */
        Eigen::Quaterniond q;
        double sigma_ecef_x;
        double sigma_ecef_y;
        double sigma_ecef_z;
        double sigma_lat,sigma_lon,sigma_alt;
        double sigma_vx, sigma_vy, sigma_vz; /*北方向速度标准差 东方向速度标准差 天顶方向速度标准差*/
    };

    /// 检查NMEA字符串的校验码
    bool checksum(const std::string &nmea);
    /// 解析发布的NMEA字符串为GNSS定位消息
    bool parse_pub_nmea(const std::string &nmea, NavSatFixMsg &gnss_pos_msg);

    /// 解析双天线rtk发布的6DOF AGRICA位置消息
    bool parse_pub_sixdof_pos(const std::string& unicore,sixdof_pose_with_cov& sixdof_pos_t);

    bool return_sentence(std::string& buffer,std::string& sentence);
}

