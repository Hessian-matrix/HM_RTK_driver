#include "HM_RTK/utils.hpp"
#include <vector>
#include <chrono>




namespace Hessian{
    
    //校验GGA字符串
    bool checksum(const std::string &nmea)
    {
        unsigned int len = nmea.length();

        if (len < 6)
            return false;

        int pos_asterisk = nmea.find("*");
        if (pos_asterisk == std::string::npos)
            return false;

        char nmea_sum[3];
        nmea_sum[0] = nmea[pos_asterisk + 1];
        nmea_sum[1] = nmea[pos_asterisk + 2];
        nmea_sum[2] = '\0';

        unsigned int sum = strtol(nmea_sum, nullptr, 16); //string to long
        unsigned int sum_ = 0;							  //calculated checksum

        for (int i = 1; i < len - 5; i++)
        {
            sum_ ^= nmea[i];
        }

        if(sum!=sum_) printf("receive check:%02x cac is:%02x\n",sum, sum_);//收到的校验和 计算校验和 截取字串

        return sum == sum_;
    }

    double GGATime2Local(const std::string& time){
        if (!boost::regex_match(time, boost::regex("^\\d{6}\\.?\\d*$"))) {
            return -1.0; // Invalid time format
        }
    
        // 获取当前时间的时区偏移量（秒）
        auto now_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        struct tm local_tm, utc_tm;
        localtime_r(&now_time, &local_tm);
        int local_offset = local_tm.tm_gmtoff;
        auto utc_time = now_time - local_offset;
        localtime_r(&utc_time, &utc_tm);
    
        // 使用tm结构保存时间
        struct tm gga_tm = utc_tm;
        double seconds = 0.0;
        try {
            gga_tm.tm_hour = std::stoi(time.substr(0, 2));
            gga_tm.tm_min = std::stoi(time.substr(2, 2));
            gga_tm.tm_sec = std::stoi(time.substr(4, 2));
            seconds = std::stod(time.substr(4)) - gga_tm.tm_sec;
        } catch(const std::exception& e) {
            std::cerr << "Invalid GGA time format: " << time << std::endl;
            return -2.0;
        }
        time_t gga_time = mktime(&gga_tm) + local_offset;
        double gga_time_local = gga_time + seconds;
    
        // Calculate the difference between the current time and the GGA time
        double time_difference = std::difftime(gga_time, now_time);
        std::cout << std::setprecision(16) << "gga time=" << gga_time_local << " now=" << now_time << ", timezone_offset=" << local_offset << ", diff=" << time_difference << " str=" << time << std::endl; 
    
        // Ensure the time difference is not excessively large
        // if (std::abs(time_difference) > 60) {
        //     return -3.0; // Invalid time
        // }
    
        // 获取gga_time的UTC时间戳，也就是1970年1月1日0时0分0秒到gga_time的秒数
        return gga_time_local;
    }

    /// @brief 解析GGA字符串，获得经纬度给ntrip client
    /// @param nmea GGA字符串
    /// @param lon_nmea 
    /// @param lat_nmea 
    /// @return 
    bool parse_pub_nmea(const std::string& nmea, NavSatFixMsg& gnss_pos_msg)
    {
        std::vector<std::string> nmea_split;
        boost::split(nmea_split, nmea, boost::is_any_of(","));
        if (nmea_split.size() < 15){
            std::cerr << "NMEA Sentence Split Failed!" << std::endl;
            return false;
        }
        
        // 检查关键字段有效性
        auto check_field = [](const std::string& field) {
            return !field.empty() && field != "0";
        };

        if(!check_field(nmea_split[2]) || !check_field(nmea_split[4])) {
            std::cerr << "NMEA Sentence Field Check Failed!" << std::endl;
            return false;
        }

        try{
            double degree, minute;
            degree = (int)strtod(nmea_split[2].c_str(), nullptr) / 100;
            minute = strtod(nmea_split[2].c_str(), nullptr) - degree * 100;
            double lat_sign = (nmea_split[3] == "S") ? -1.0 : 1.0;
            gnss_pos_msg.latitude = lat_sign *(degree + minute / 60.0);

            degree = (int)strtod(nmea_split[4].c_str(), nullptr) / 100;
            minute = strtod(nmea_split[4].c_str(), nullptr) - degree * 100;
            double lon_sign = (nmea_split[5] == "W") ? -1.0 : 1.0;
            gnss_pos_msg.longitude = lon_sign * (degree + minute / 60.0);

            gnss_pos_msg.altitude = strtod(nmea_split[9].c_str(), nullptr) + strtod(nmea_split[11].c_str(), nullptr);

            int satnu = strtol(nmea_split[7].c_str(), nullptr, 10);
            gnss_pos_msg.position_covariance_type = satnu;

            double hdop = strtod(nmea_split[8].c_str(), nullptr);
            double pos_cov = hdop * hdop / 2.0;
            gnss_pos_msg.position_covariance = { 
                pos_cov, 0.0, 0.0,
                0.0, pos_cov, 0.0,
                0.0, 0.0, pos_cov 
            };

#ifdef ROS1_BUILD
            sensor_msgs::NavSatStatus gnss_pos_status;
#else
            sensor_msgs::msg::NavSatStatus gnss_pos_status;
#endif
            int pos_status = strtol(nmea_split[6].c_str(), nullptr, 10);
            if (pos_status == 0)
                gnss_pos_status.status = -1;
            else if (pos_status == 1)
                gnss_pos_status.status = 0;
            else if (pos_status == 2 || pos_status == 5)
                gnss_pos_status.status = 1;
            else if (pos_status == 4)
                gnss_pos_status.status = 2;

            gnss_pos_msg.status = gnss_pos_status;
            double time_local = GGATime2Local(nmea_split[1]);
            if(time_local < 0) {
                std::cerr << "NMEA Sentence Time Parse Failed!,local time="<<time_local << std::endl;
                return false;
            }
#ifdef ROS1_BUILD
            gnss_pos_msg.header.stamp.fromSec(time_local);
#else
            auto sec = static_cast<int32_t>(time_local);
            auto nanosec = static_cast<uint32_t>((time_local - sec) * 1e9);
            gnss_pos_msg.header.stamp.sec = sec;
            gnss_pos_msg.header.stamp.nanosec = nanosec;
#endif
            gnss_pos_msg.header.frame_id = "rtk_link";
            // std::cout << "rtk time=" << gnss_pos_msg.header.timestamp << " cur time=" << rosTime
		    // 	<< " diff=" << rosTime - gnss_pos_msg.header.timestamp << std::endl;
        }catch(...){
            std::cerr << "NMEA Sentence Parse Failed!" << std::endl;
            return false;
        }

        return true;
    }

    bool parse_pub_sixdof_pos(const std::string& unicore,sixdof_pose_with_cov& sixdof_pos_t)
    {
        const auto it = unicore.find(";GNSS");
        const auto it_end = unicore.find("*");
        if (it == std::string::npos && it_end ==std::string::npos){
            std::cerr<<"not find AGRIC\n"<<std::endl;
            return false;
        }
        std::string core_str(unicore.begin() + it,unicore.end());
        // std::cout<<"core_str "<<core_str<<std::endl;
        std::vector<std::string> unicore_split;
        boost::split(unicore_split, core_str, boost::is_any_of(","));
        if (unicore_split.size() < 57){
            std::cerr << "AGRIC Unicore Sentence Split Failed! split is "<< unicore_split.size() << std::endl;
            return false;
        }

        int year = std::strtod(unicore_split[2].c_str(),nullptr);
        int minute = std::strtod(unicore_split[6].c_str(),nullptr);
        int state = std::strtod(unicore_split[9].c_str(),nullptr);
        double yaw = std::strtod(unicore_split[19].c_str(),nullptr);
        double pitch = std::strtod(unicore_split[20].c_str(),nullptr);
        double roll = std::strtod(unicore_split[21].c_str(),nullptr);
        double ecrf_x = std::strtod(unicore_split[32].c_str(),nullptr);
        double ecrf_y = std::strtod(unicore_split[33].c_str(),nullptr);
        double ecrf_z = std::strtod(unicore_split[34].c_str(),nullptr);
        
        sixdof_pos_t.year = year;
        sixdof_pos_t.mounth = std::strtod(unicore_split[3].c_str(),nullptr);
        sixdof_pos_t.day = std::strtod(unicore_split[4].c_str(),nullptr);
        sixdof_pos_t.hour = std::strtod(unicore_split[5].c_str(),nullptr);
        sixdof_pos_t.minute = minute;//分
        sixdof_pos_t.second = std::strtod(unicore_split[7].c_str(),nullptr);//秒
        sixdof_pos_t.second_gps = std::strtod(unicore_split[47].c_str(),nullptr);/*GPS 周内毫秒*/

        sixdof_pos_t.state = (int)std::strtod(unicore_split[9].c_str(),nullptr);
        sixdof_pos_t.sat = (int)std::strtod(unicore_split[10].c_str(),nullptr);  /*参与解算 GPS 卫星数*/
        sixdof_pos_t.sat += (int)std::strtod(unicore_split[11].c_str(),nullptr); /*参与解算北斗卫星数 */
        sixdof_pos_t.sat += (int)std::strtod(unicore_split[12].c_str(),nullptr); /*参与解算 GLONASS 卫星数 */

        printf("y %d s %d sat:%d state:%d [%f %f %f],[%f %f %f]\n",year,minute,sixdof_pos_t.sat, state,yaw,pitch,roll,ecrf_x,ecrf_y,ecrf_z);

        sixdof_pos_t.ecef_x = std::strtod(unicore_split[32].c_str(),nullptr);
        sixdof_pos_t.ecef_y = std::strtod(unicore_split[33].c_str(),nullptr);
        sixdof_pos_t.ecef_z = std::strtod(unicore_split[34].c_str(),nullptr);

        //zyx顺序
        Eigen::Quaterniond q = Eigen::AngleAxisd(std::strtod(unicore_split[19].c_str(),nullptr)* M_PI /180, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(std::strtod(unicore_split[20].c_str(),nullptr)* M_PI /180, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

        sixdof_pos_t.q = q;
        sixdof_pos_t.xigema_ecef_x = std::strtod(unicore_split[38].c_str(),nullptr);
        sixdof_pos_t.xigema_ecef_y = std::strtod(unicore_split[39].c_str(),nullptr);
        sixdof_pos_t.xigema_ecef_z = std::strtod(unicore_split[40].c_str(),nullptr);
        sixdof_pos_t.xigema_lat = std::strtod(unicore_split[35].c_str(),nullptr);
        sixdof_pos_t.xigema_lon = std::strtod(unicore_split[36].c_str(),nullptr),/*经度标准差 */
        sixdof_pos_t.xigema_alt = std::strtod(unicore_split[37].c_str(),nullptr),/*高程标准差*/

        sixdof_pos_t.lat = std::strtod(unicore_split[29].c_str(),nullptr);
        sixdof_pos_t.lon = std::strtod(unicore_split[30].c_str(),nullptr);/*经纬高 lon*/
        sixdof_pos_t.alt = std::strtod(unicore_split[31].c_str(),nullptr);/*经纬高 alt*/

        //东北天速度
        sixdof_pos_t.v_north = std::strtod(unicore_split[23].c_str(),nullptr);/*北方向速度*/
        sixdof_pos_t.v_east = std::strtod(unicore_split[24].c_str(),nullptr);/*东方向速度 */
        sixdof_pos_t.v_up = std::strtod(unicore_split[25].c_str(),nullptr);/*天顶方向速度 */
        


        sixdof_pos_t.xigema_vx = std::strtod(unicore_split[26].c_str(),nullptr);/*北方向速度标准差*/
        sixdof_pos_t.xigema_vx = std::strtod(unicore_split[27].c_str(),nullptr);/*东方向速度标准差*/
        sixdof_pos_t.xigema_vz = std::strtod(unicore_split[28].c_str(),nullptr);/*天顶方向速度标准差*/

        sixdof_pos_t.pose_type = std::strtod(unicore_split[10].c_str(),nullptr);/*流动站定位状态：*/
        sixdof_pos_t.speed_type = std::strtod(unicore_split[54].c_str(),nullptr);/*速度解状态有效*/

        return true;
    }

}