#include "HM_RTK/utils.hpp"
#include "HM_RTK/serial_hm.hpp"
#include "HM_RTK/ros_adapter.hpp"
#include "ntrip/ntrip_client.h"

Hessian::Serial hm_serial;
ros_adapter::Publisher_t<StringMsg> pub_rtk_nmea;
ros_adapter::Publisher_t<PoseStampedMsg> pub_ex_pose;
libntrip::NtripClient ntrip_client;

Eigen::Vector3d ex_rtk_slam(0, 0, 0);
std::atomic<bool> stop_ex_publish(false);

using namespace Hessian;

void publishExPose(ros_adapter::NodeHandle nh) {
    ros_adapter::Rate rate(1.0);
    while (ros_adapter::ok() && !stop_ex_publish) {
        try {
            PoseStampedMsg pose;
#ifdef ROS1_BUILD
            pose.header.stamp = ros_adapter::now();
#else
            pose.header.stamp = ros_adapter::now(nh);
#endif
            pose.header.frame_id = "rtk";
            pose.pose.position.x = ex_rtk_slam.x();
            pose.pose.position.y = ex_rtk_slam.y();
            pose.pose.position.z = ex_rtk_slam.z();
            pose.pose.orientation.w = 1.0;
            pub_ex_pose->publish(pose);
            rate.sleep();
        } catch (const ros_adapter::Exception& e) {
            ROS_ERROR_STREAM("Ex pose publisher error: " << e.what());
        }
    }
}

// 主函数修改
int main(int argc, char **argv) {
    ros_adapter::init(argc, argv, "HM_RTK");
    
#ifdef ROS1_BUILD
    ros_adapter::NodeHandle nh = ros_adapter::createNodeHandle("~");
#else
    auto nh = ros_adapter::createNodeHandle("hm_rtk");
#endif

    // 从参数服务器读取参数
    std::string ntrip_ip, ntrip_user, ntrip_passwd, ntrip_mountpoint, rtk_port;
    int ntrip_port, rtk_baudrate;
    std::string pub_rtk_nmea_topic, pub_rtk_ex_pose_topic;
    
    ros_adapter::getParam(nh, "ntrip_ip", ntrip_ip, std::string("127.0.0.1"));
    ros_adapter::getParam(nh, "ntrip_port", ntrip_port, 8002);
    ros_adapter::getParam(nh, "ntrip_user", ntrip_user, std::string("user"));
    ros_adapter::getParam(nh, "ntrip_passwd", ntrip_passwd, std::string("password"));
    ros_adapter::getParam(nh, "ntrip_mountpoint", ntrip_mountpoint, std::string("RTCM33_GRCEJ"));
    ros_adapter::getParam(nh, "rtk_port", rtk_port, std::string("/dev/ttyUSB0"));
    ros_adapter::getParam(nh, "rtk_baudrate", rtk_baudrate, 115200);
    ros_adapter::getParam(nh, "pub_rtk_nmea_topic", pub_rtk_nmea_topic, std::string("/rtk_nmea"));
    ros_adapter::getParam(nh, "pub_rtk_ex_pose_topic", pub_rtk_ex_pose_topic, std::string("/rtk_extrinsic"));

    double ex_x, ex_y, ex_z;
    ros_adapter::getParam(nh, "ex_rtk_slam_x", ex_x, 0.0);
    ros_adapter::getParam(nh, "ex_rtk_slam_y", ex_y, 0.0);
    ros_adapter::getParam(nh, "ex_rtk_slam_z", ex_z, 0.0);
    ex_rtk_slam = Eigen::Vector3d(ex_x, ex_y, ex_z);

    pub_rtk_nmea = ros_adapter::advertise<StringMsg>(nh, pub_rtk_nmea_topic, 5);
    pub_ex_pose = ros_adapter::advertise<PoseStampedMsg>(nh, pub_rtk_ex_pose_topic, 5);

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
#ifdef ROS1_BUILD
            ros_adapter::sleep_duration(3.0).sleep();
#else
            ros_adapter::sleep_for(3.0);
#endif
            if (!ntrip_client.Run()) {
                ROS_FATAL("NTRIP client initialization failed!");
                return EXIT_FAILURE;
            }
        }
        ROS_INFO_STREAM("NTRIP client status: " << ntrip_client.service_is_running());
	}

    // 启动外参发布线程
    stop_ex_publish.store(false);
    std::thread ex_publish_thread(publishExPose, nh);

    int f_count=0;
    while(ros_adapter::ok())
    {
        try {
            std::string c = hm_serial.read(1);
            if (c.empty()) {
#ifdef ROS1_BUILD
                ros_adapter::sleep_duration(0.001).sleep();
#else
                ros_adapter::sleep_for(0.001);
#endif
                continue;
            }

            if(c=="$"){
                c=hm_serial.read(1);
                if(c=="G")
                {
                    std::string ret = hm_serial.readline(128);
                    std::string nmea = "$G" + ret;
                    if(nmea.find("GGA") == std::string::npos && nmea.find("RMC") == std::string::npos)
                        continue;
                    bool is_nmea = checksum(nmea);//检查校验和
                    if (!is_nmea)
                    {
                        std::cerr<<"NMEA Sentence Check Failed!"<<std::endl;
                        std::cout<<"\033[31m"<<nmea<<"\033[0m"<<std::endl;
                        continue;
                    }
                    
                    if(nmea.find("RMC") != std::string::npos)
                    {
                        StringMsg msg;
                        msg.data = nmea;
                        pub_rtk_nmea->publish(msg);
                        std::cout<<nmea;
                    }                    

                    if (nmea.find("GGA") != std::string::npos){
                        NavSatFixMsg gnss_pos_msg;
                        bool ret = parse_pub_nmea(nmea, gnss_pos_msg);
                        if(!ret) { continue; }
                        // std::string time_str = "@" + std::to_string(time_now);
                        // double time_now = ros::Time::now().toNSec() / 1e9;
                        // nmea.insert(nmea.size() -2,time_str);
                        // nmea = nmea + "@" + std::to_string(time_now);//测试 打上系统时间
                        StringMsg msg;
                        msg.data = nmea;
                        pub_rtk_nmea->publish(msg);	//发布GGA字符串
                        ntrip_client.set_location(gnss_pos_msg.latitude, gnss_pos_msg.longitude);
                        std::cout << nmea;
                    }
                }
            }
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Serial read error: " << e.what());
#ifdef ROS1_BUILD
            ros_adapter::sleep_duration(1.0).sleep();
#else
            ros_adapter::sleep_for(1.0);
#endif
        }
#ifdef ROS1_BUILD
        ros_adapter::spinOnce();
#else
        ros_adapter::spinOnce(nh);
#endif
    }
    stop_ex_publish.store(true);
    if(ex_publish_thread.joinable())
        ex_publish_thread.join();
    return 0;
}


