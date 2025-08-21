#ifndef ROS_ADAPTER_HPP
#define ROS_ADAPTER_HPP

#ifdef ROS1_BUILD
    #include <ros/ros.h>
    #include <std_msgs/String.h>
    #include <geometry_msgs/PoseStamped.h>
    #include <sensor_msgs/NavSatFix.h>
    #include <nav_msgs/Odometry.h>
    
    namespace ros_adapter {
        using NodeHandle = ros::NodeHandle;
        using Publisher = ros::Publisher;
        using Subscriber = ros::Subscriber;
        using Rate = ros::Rate;
        using Time = ros::Time;
        using Duration = ros::Duration;
        using Exception = ros::Exception;
        
        template<typename T>
        using Publisher_t = ros::Publisher;
        
        template<typename T>
        using Subscriber_t = ros::Subscriber;
        
        inline bool ok() { return ros::ok(); }
        inline void spinOnce() { ros::spinOnce(); }
        inline void init(int argc, char** argv, const std::string& name) {
            ros::init(argc, argv, name);
        }
        
        template<typename T>
        void getParam(NodeHandle& nh, const std::string& key, T& value, const T& default_value) {
            nh.param<T>(key, value, default_value);
        }
        
        inline NodeHandle createNodeHandle(const std::string& ns = "~") {
            return NodeHandle(ns);
        }
        
        template<typename T>
        Publisher_t<T> advertise(NodeHandle& nh, const std::string& topic, int queue_size) {
            return nh.advertise<T>(topic, queue_size);
        }
        
        template<typename T, typename CallbackT>
        Subscriber_t<T> subscribe(NodeHandle& nh, const std::string& topic, int queue_size, CallbackT callback) {
            return nh.subscribe<T>(topic, queue_size, callback);
        }
        
        inline Time now() { return ros::Time::now(); }
        inline Duration sleep_duration(double seconds) { return ros::Duration(seconds); }
        
        #define ROS_INFO_STREAM(x) ROS_INFO_STREAM(x)
        #define ROS_ERROR_STREAM(x) ROS_ERROR_STREAM(x)
        #define ROS_FATAL_STREAM(x) ROS_FATAL_STREAM(x)
        #define ROS_WARN_STREAM(x) ROS_WARN_STREAM(x)
        #define ROS_INFO(x) ROS_INFO(x)
        #define ROS_ERROR(x) ROS_ERROR(x)
        #define ROS_FATAL(x) ROS_FATAL(x)
        #define ROS_WARN(x) ROS_WARN(x)
    }

#elif defined(ROS2_BUILD)
    #include <rclcpp/rclcpp.hpp>
    #include <std_msgs/msg/string.hpp>
    #include <geometry_msgs/msg/pose_stamped.hpp>
    #include <sensor_msgs/msg/nav_sat_fix.hpp>
    #include <nav_msgs/msg/odometry.hpp>
    #include <chrono>
    
    namespace ros_adapter {
        using NodeHandle = std::shared_ptr<rclcpp::Node>;
        using Rate = rclcpp::Rate;
        using Time = rclcpp::Time;
        using Duration = rclcpp::Duration;
        using Exception = std::runtime_error;
        
        template<typename T>
        using Publisher_t = typename rclcpp::Publisher<T>::SharedPtr;
        
        template<typename T>
        using Subscriber_t = typename rclcpp::Subscription<T>::SharedPtr;
        
        inline bool ok() { return rclcpp::ok(); }
        inline void spinOnce(NodeHandle& nh) { rclcpp::spin_some(nh); }
        inline void spinOnce() { 
            // 对于ROS2，需要传入节点，但为了兼容性，这里实现一个空函数
        }
        
        inline void init(int argc, char** argv, const std::string& name) {
            rclcpp::init(argc, argv);
        }
        
        template<typename T>
        void getParam(NodeHandle& nh, const std::string& key, T& value, const T& default_value) {
            // 对于ROS2，首先尝试获取已存在的参数，如果不存在则声明
            if (!nh->has_parameter(key)) {
                nh->declare_parameter(key, default_value);
            }
            value = nh->get_parameter(key).get_value<T>();
        }
        
        inline NodeHandle createNodeHandle(const std::string& name = "hm_rtk") {
            return rclcpp::Node::make_shared(name);
        }
        
        template<typename T>
        Publisher_t<T> advertise(NodeHandle& nh, const std::string& topic, int queue_size) {
            return nh->create_publisher<T>(topic, queue_size);
        }
        
        template<typename T, typename CallbackT>
        Subscriber_t<T> subscribe(NodeHandle& nh, const std::string& topic, int queue_size, CallbackT callback) {
            return nh->create_subscription<T>(topic, queue_size, callback);
        }
        
        inline Time now(NodeHandle& nh) { return nh->now(); }
        inline Time now() { return rclcpp::Clock().now(); }
        inline Duration sleep_duration(double seconds) { 
            return rclcpp::Duration::from_seconds(seconds); 
        }
        
        inline void sleep_for(double seconds) {
            rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(seconds * 1000)));
        }
        
        // 时间相关辅助函数
        inline double toSec(const rclcpp::Time& time) {
            return time.seconds();
        }
        
        inline double stampToSec(const builtin_interfaces::msg::Time& stamp) {
            return stamp.sec + stamp.nanosec / 1e9;
        }
        
        inline void fromSec(double time_sec, builtin_interfaces::msg::Time& stamp) {
            stamp.sec = static_cast<int32_t>(time_sec);
            stamp.nanosec = static_cast<uint32_t>((time_sec - stamp.sec) * 1e9);
        }
        
        #define ROS_INFO_STREAM(x) RCLCPP_INFO_STREAM(rclcpp::get_logger("hm_rtk"), x)
        #define ROS_ERROR_STREAM(x) RCLCPP_ERROR_STREAM(rclcpp::get_logger("hm_rtk"), x)
        #define ROS_FATAL_STREAM(x) RCLCPP_FATAL_STREAM(rclcpp::get_logger("hm_rtk"), x)
        #define ROS_WARN_STREAM(x) RCLCPP_WARN_STREAM(rclcpp::get_logger("hm_rtk"), x)
        #define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("hm_rtk"), __VA_ARGS__)
        #define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("hm_rtk"), __VA_ARGS__)
        #define ROS_FATAL(...) RCLCPP_FATAL(rclcpp::get_logger("hm_rtk"), __VA_ARGS__)
        #define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("hm_rtk"), __VA_ARGS__)
    }
#endif

// 消息类型定义
#ifdef ROS1_BUILD
    using StringMsg = std_msgs::String;
    using PoseStampedMsg = geometry_msgs::PoseStamped;
    using NavSatFixMsg = sensor_msgs::NavSatFix;
    using OdometryMsg = nav_msgs::Odometry;
    using HeaderMsg = std_msgs::Header;
    
    // 时间戳访问宏
    #define GET_STAMP_SEC(msg) ((msg).header.stamp.toSec())
    #define SET_STAMP_SEC(msg, sec) ((msg).header.stamp.fromSec(sec))
    
#elif defined(ROS2_BUILD)
    using StringMsg = std_msgs::msg::String;
    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
    using NavSatFixMsg = sensor_msgs::msg::NavSatFix;
    using OdometryMsg = nav_msgs::msg::Odometry;
    using HeaderMsg = std_msgs::msg::Header;
    
    // 时间戳访问宏
    #define GET_STAMP_SEC(msg) (ros_adapter::stampToSec((msg).header.stamp))
    #define SET_STAMP_SEC(msg, sec) (ros_adapter::fromSec(sec, (msg).header.stamp))
    
#endif

#endif // ROS_ADAPTER_HPP
