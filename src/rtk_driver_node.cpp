#include "HM_RTK/utils.hpp"
#include "HM_RTK/serial_hm.hpp"
#include "ntrip/ntrip_client.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>



Hessian::Serial hm_serial;
ros::Publisher pub_rtk_nmea, pub_ex_pose;
libntrip::NtripClient ntrip_client;

Eigen::Vector3d ex_rtk_slam(0, 0, 0);
std::atomic<bool> stop_ex_publish(false);

using namespace Hessian;

void publishExPose() {
    ros::Rate rate(1.0);
    while (ros::ok() && !stop_ex_publish) {
        try {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "rtk";
            pose.pose.position.x = ex_rtk_slam.x();
            pose.pose.position.y = ex_rtk_slam.y();
            pose.pose.position.z = ex_rtk_slam.z();
            pose.pose.orientation.w = 1.0;  // 单位四元数
            pub_ex_pose.publish(pose);
            rate.sleep();
        } catch (const ros::Exception& e) {
            ROS_ERROR_STREAM("Ex pose publisher error: " << e.what());
        }
    }
}

// 主函数修改
int main(int argc, char **argv) {
    ros::init(argc, argv, "HM_RTK");
    ros::NodeHandle nh("~");

    // 从参数服务器读取参数
    std::string ntrip_ip, ntrip_user, ntrip_passwd, ntrip_mountpoint, rtk_port;
    int ntrip_port, rtk_baudrate;
    std::string pub_rtk_nmea_topic, pub_rtk_ex_pose_topic;
    nh.param<std::string>("ntrip_ip", ntrip_ip, "127.0.0.1");
    nh.param<int>("ntrip_port", ntrip_port, 8002);
    nh.param<std::string>("ntrip_user", ntrip_user, "user");
    nh.param<std::string>("ntrip_passwd", ntrip_passwd, "password");
    nh.param<std::string>("ntrip_mountpoint", ntrip_mountpoint, "RTCM33_GRCEJ");
    nh.param<std::string>("rtk_port", rtk_port, "/dev/ttyUSB0");
    nh.param<int>("rtk_baudrate", rtk_baudrate, 115200);
    nh.param<std::string>("pub_rtk_nmea_topic",pub_rtk_nmea_topic,"/rtk_nmea");
    nh.param<std::string>("pub_rtk_ex_pose_topic",pub_rtk_ex_pose_topic,"/rtk_extrinsic");

    nh.param<double>("ex_rtk_slam_x", ex_rtk_slam.x(), 0.0);
    nh.param<double>("ex_rtk_slam_y", ex_rtk_slam.y(), 0.0);
    nh.param<double>("ex_rtk_slam_z", ex_rtk_slam.z(), 0.0);

    pub_rtk_nmea = nh.advertise<std_msgs::String>(pub_rtk_nmea_topic,5);
    pub_ex_pose = nh.advertise<geometry_msgs::PoseStamped>(pub_rtk_ex_pose_topic, 5);

    ROS_INFO_STREAM("NTRIP IP: " << ntrip_ip);
    ROS_INFO_STREAM("NTRIP Port: " << ntrip_port);
    ROS_INFO_STREAM("NTRIP User: " << ntrip_user);
    ROS_INFO_STREAM("NTRIP Password: " << ntrip_passwd);
    ROS_INFO_STREAM("NTRIP Mountpoint: " << ntrip_mountpoint);
    ROS_INFO_STREAM("RTK Port: " << rtk_port);
    ROS_INFO_STREAM("RTK Baudrate: " << rtk_baudrate);

    ROS_INFO_STREAM("Ex RTK-SLAM: " << ex_rtk_slam.transpose());

    // 参数有效性检查
    if (rtk_port.empty()) {
        ROS_FATAL("Invalid serial port configuration!");
        return EXIT_FAILURE;
    }

	// 配置串口
    try {
        hm_serial.reset(rtk_port, rtk_baudrate);
        if(!hm_serial.isOpen()) {
            ROS_FATAL_STREAM("GNSS port [" << rtk_port << "] open failed!");
            return EXIT_FAILURE;
        }
    } catch (const std::exception& e) {
        ROS_FATAL_STREAM("Serial exception: " << e.what());
        return EXIT_FAILURE;
    }

	// Ntrip 服务
	if(!ntrip_mountpoint.empty()){
		ntrip_client.Init(ntrip_ip, ntrip_port, ntrip_user, ntrip_passwd, ntrip_mountpoint);
		ntrip_client.OnReceived([] (const char *buffer, int size) {
			int ret = hm_serial.write( std::string(buffer, size));
			std::cout << "serial try to write:" << size << ", real write=" << ret << ", drop=" << size - ret << std::endl;
		});
		if (!ntrip_client.Run()) {
            ROS_ERROR("NTRIP client start failed! Retrying in 3 seconds...");
            ros::Duration(3.0).sleep();
            if (!ntrip_client.Run()) {
                ROS_FATAL("NTRIP client initialization failed!");
                return EXIT_FAILURE;
            }
        }
        ROS_INFO_STREAM("NTRIP client status: " << ntrip_client.service_is_running());
	}

    // 启动外参发布线程
    stop_ex_publish.store(false);
    std::thread ex_publish_thread(publishExPose);


    while(ros::ok())
    {
        try {
            std::string c = hm_serial.read(1);
            if (c.empty()) {
                ros::Duration(0.001).sleep();
                continue;
            }

            if(c=="$"){
                c=hm_serial.read(1);
                if(c=="G")
                {
                    std::string ret = hm_serial.readline(128);
                    std::string nmea = "$G" + ret;
                    bool is_nmea = checksum(nmea);//检查校验和
                    if (!is_nmea)
                    {
                        std::cerr<<"NMEA Sentence Check Failed!"<<std::endl;
                        continue;
                    }

                    if (nmea.find("GGA") != std::string::npos){
                        sensor_msgs::NavSatFix gnss_pos_msg;
                        bool ret = parse_pub_nmea(nmea, gnss_pos_msg);
                        if(!ret) { continue; }
                        
                        std_msgs::String msg;
                        msg.data = nmea;
                        pub_rtk_nmea.publish(msg);	//发布GGA字符串
                        ntrip_client.set_location(gnss_pos_msg.latitude, gnss_pos_msg.longitude);
                        std::cout << nmea;
                    }
                }
            }
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Serial read error: " << e.what());
            ros::Duration(1.0).sleep();  // 错误恢复等待
        }
        ros::spinOnce();
    }
    stop_ex_publish.store(true);
    if(ex_publish_thread.joinable())
        ex_publish_thread.join();
    return 0;
}


