#pragma once
#include <iostream>
#include <string>
#include <sensor_msgs/NavSatFix.h>
#include <Eigen/Core>

// Namespace Hessian 提供NMEA消息解析等实用函数
namespace Hessian {
    /// 检查NMEA字符串的校验码
    bool checksum(const std::string &nmea);
    /// 解析发布的NMEA字符串为GNSS定位消息
    bool parse_pub_nmea(const std::string &nmea, sensor_msgs::NavSatFix &gnss_pos_msg);
}

