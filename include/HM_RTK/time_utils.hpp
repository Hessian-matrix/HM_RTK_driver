#ifndef TIME_UTILS_HPP
#define TIME_UTILS_HPP

#include "ros_adapter.hpp"

namespace time_utils {
    
#ifdef ROS1_BUILD
    inline double toSec(const ros::Time& time) {
        return time.toSec();
    }
    
    inline double stampToSec(const std_msgs::Header& header) {
        return header.stamp.toSec();
    }
    
    inline void secToStamp(double time_sec, std_msgs::Header& header) {
        header.stamp.fromSec(time_sec);
    }
#else
    inline double toSec(const rclcpp::Time& time) {
        return time.seconds();
    }
    
    inline double stampToSec(const std_msgs::msg::Header& header) {
        return header.stamp.sec + header.stamp.nanosec / 1e9;
    }
    
    inline void secToStamp(double time_sec, std_msgs::msg::Header& header) {
        header.stamp.sec = static_cast<int32_t>(time_sec);
        header.stamp.nanosec = static_cast<uint32_t>((time_sec - header.stamp.sec) * 1e9);
    }
#endif

}

#endif // TIME_UTILS_HPP
