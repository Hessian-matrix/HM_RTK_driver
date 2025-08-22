#include "HM_RTK/utils.hpp"
#include "HM_RTK/serial_hm.hpp"
#include "ntrip/ntrip_client.h"

using namespace Hessian;



class HM_RTK_Node : public rclcpp::Node{
private:
    rclcpp::Publisher<string_msg>::SharedPtr pub_rtk_nmea;
    rclcpp::Publisher<poseStamped_msg>::SharedPtr pub_ex_pose;

    libntrip::NtripClient ntrip_client;
    Eigen::Vector3d ex_rtk_slam;
    std::atomic<bool> stop_ex_publish;
    Hessian::Serial hm_serial;

    //params
    std::string ntrip_ip;
    int ntrip_port;
    std::string ntrip_user;
    std::string ntrip_passwd;
    std::string ntrip_mountpoint;
    std::string rtk_port;
    int rtk_baudrate;
    std::string pub_rtk_nmea_topic;
    std::string pub_rtk_ex_pose_topic; 
    std::string rtk_sixdof_6axis_topic;

    std::thread ex_publish_thread;
    std::thread serial_loop_thread;


public:
    HM_RTK_Node(std::string name) : Node(name){
        read_params();
        printf("start thread\n");
        // 启动外参发布线程
        stop_ex_publish.store(false);
        ex_publish_thread =std::thread(&HM_RTK_Node::publishExPose,this);
        serial_loop_thread = std::thread(&HM_RTK_Node::loop,this);
    }
    ~HM_RTK_Node()
    {
        hm_serial.close();
        ex_publish_thread.join();
        serial_loop_thread.join();
    }

    void publishExPose();
    
    void read_params();

    int loop();
    rclcpp::Logger logger()
    {
        return this->get_logger();
    }


};


void HM_RTK_Node::publishExPose(){
    rclcpp::Rate rate(1.0);
    while (rclcpp::ok() && !stop_ex_publish) {
        try {
            poseStamped_msg pose;
            
            pose.header.frame_id = "rtk";
            pose.pose.position.x = ex_rtk_slam.x();
            pose.pose.position.y = ex_rtk_slam.y();
            pose.pose.position.z = ex_rtk_slam.z();
            pose.pose.orientation.w = 1.0;  // 单位四元数
            #ifdef ROS1
            pose.header.stamp = ros::Time::now();
            pub_ex_pose.publish(pose);
            #endif
            #ifdef ROS2
            pose.header.stamp = this->now();
            pub_ex_pose->publish(pose);
            #endif
            rate.sleep();
        } catch (const std::exception& e) {
            std::cerr << "Ex pose publisher error: " << e.what()<<std::endl;
        }
    }
}

void HM_RTK_Node::read_params(){
    //get param
    this->declare_parameter<std::string>("ntrip_ip", "127.0.0.1");
    this->declare_parameter<int>("ntrip_port", 8002);
    this->declare_parameter<std::string>("ntrip_user", "user");
    this->declare_parameter<std::string>("ntrip_passwd", "password");
    this->declare_parameter<std::string>("ntrip_mountpoint", "RTCM33_GRCEJ");
    this->declare_parameter<std::string>("rtk_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("rtk_baudrate", 115200);
    this->declare_parameter<std::string>("pub_rtk_nmea_topic", "/rtk_nmea");
    this->declare_parameter<std::string>("pub_rtk_ex_pose_topic", "/rtk_extrinsic");
    this->declare_parameter<double>("ex_rtk_slam_x", 0.0);
    this->declare_parameter<double>("ex_rtk_slam_y", 0.0);
    this->declare_parameter<double>("ex_rtk_slam_z", 0.0);



    // 获取参数值
    ntrip_ip = this->get_parameter("ntrip_ip").as_string();
    ntrip_port = this->get_parameter("ntrip_port").as_int();
    ntrip_user = this->get_parameter("ntrip_user").as_string();
    ntrip_passwd = this->get_parameter("ntrip_passwd").as_string();
    ntrip_mountpoint = this->get_parameter("ntrip_mountpoint").as_string();
    rtk_port = this->get_parameter("rtk_port").as_string();
    rtk_baudrate = this->get_parameter("rtk_baudrate").as_int();
    pub_rtk_nmea_topic = this->get_parameter("pub_rtk_nmea_topic").as_string();
    pub_rtk_ex_pose_topic = this->get_parameter("pub_rtk_ex_pose_topic").as_string();
    
    ex_rtk_slam.x() = this->get_parameter("ex_rtk_slam_x").as_double();
    ex_rtk_slam.y() = this->get_parameter("ex_rtk_slam_y").as_double();
    ex_rtk_slam.z() = this->get_parameter("ex_rtk_slam_z").as_double();

    pub_rtk_nmea = this->create_publisher<string_msg>(pub_rtk_nmea_topic,5);
    pub_ex_pose = this->create_publisher<poseStamped_msg>(pub_rtk_ex_pose_topic,5);

    RCLCPP_INFO(logger(),"serial port:%s baudrate:%d",rtk_port.c_str(),rtk_baudrate);
    RCLCPP_INFO(logger(),"ntrip:%s, user:%s mountpoint:%s",ntrip_ip.c_str(),ntrip_user.c_str(),ntrip_mountpoint.c_str());

}

int HM_RTK_Node::loop()
{
    // 参数有效性检查
    if (rtk_port.empty()) {
        std::cerr<<"Invalid serial port configuration!"<<std::endl;
        return EXIT_FAILURE;
    }

	// 配置串口
    try {
        hm_serial.reset(rtk_port, rtk_baudrate);
        if(!hm_serial.isOpen()) {
            std::cerr<<"GNSS port [" << rtk_port << "] open failed!"<<std::endl;
            return EXIT_FAILURE;
        }
    } catch (const std::exception& e) {
        std::cerr<<"Serial exception: " << e.what()<<std::endl;
        return EXIT_FAILURE;
    }

	// Ntrip 服务
	if(!ntrip_mountpoint.empty()){
		ntrip_client.Init(ntrip_ip, ntrip_port, ntrip_user, ntrip_passwd, ntrip_mountpoint);
		ntrip_client.OnReceived([this] (const char *buffer, int size) {
			int ret = hm_serial.write( std::string(buffer, size));
			std::cout << "serial try to write:" << size << ", real write=" << ret << ", drop=" << size - ret << std::endl;
		});
		if (!ntrip_client.Run()) {
            std::cerr<<"NTRIP client start failed! Retrying in 3 seconds..."<<std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(3));

            if (!ntrip_client.Run()) {
                std::cerr<<"NTRIP client initialization failed!"<<std::endl;
                return EXIT_FAILURE;
            }
        }
        std::cout<<"NTRIP client status: " << ntrip_client.service_is_running()<<std::endl;
	}
     
    // rclcpp::Rate r(100);
    int f_count=0;
    std::string bufferc;
    while(rclcpp::ok())
    {
        try {
            std::string c = hm_serial.read(1);
            if (c.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));  // 休眠1毫秒
                continue;
            }
            
            if(c=="$"){
                c=hm_serial.read(1);
                if(c=="G")
                {
                    std::string ret = hm_serial.readline(280);
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
                        string_msg msg;
                        msg.data = nmea;
                        pub_rtk_nmea->publish(msg);
                        std::cout<<nmea;
                    }                    

                    if (nmea.find("GGA") != std::string::npos){
                        NavSatFix_msg gnss_pos_msg;
                        bool ret = parse_pub_nmea(nmea, gnss_pos_msg);
                        if(!ret) { continue; }
                        // std::string time_str = "@" + std::to_string(time_now);
                        // double time_now = ros::Time::now().toNSec() / 1e9;
                        // nmea.insert(nmea.size() -2,time_str);
                        // nmea = nmea + "@" + std::to_string(time_now);//测试 打上系统时间
                        string_msg msg;
                        msg.data = nmea;
                        pub_rtk_nmea->publish(msg);	//发布GGA字符串
                        ntrip_client.set_location(gnss_pos_msg.latitude, gnss_pos_msg.longitude);
                        std::cout << nmea;

                    }
                }
            }
            if(c == "#")
            {
                std::string ret = hm_serial.readline(500);
                std::string unicore = "#" + ret;
                if(unicore.find("AGRICA") != std::string::npos)
                {
                    string_msg msg;
                    msg.data = unicore;
                    pub_rtk_nmea->publish(msg);
                    // std::cout<<unicore;
                }
            }
        } catch (const std::exception& e) {
            // ROS_ERROR_STREAM("Serial read error: " << e.what());
            std::cerr<<"Serial read error: " << e.what()<<std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        
        // r.sleep();
    }
    return 0;
   
}









// 主函数修改
int main(int argc, char **argv) {
    
    rclcpp::init(argc,argv);
    auto node = std::make_shared<HM_RTK_Node>("HM_RTK");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


