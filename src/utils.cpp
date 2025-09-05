#include "HM_RTK/utils.hpp"
#include <vector>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <chrono>
#include <exception>
#include <boost/regex.hpp>

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

        if(sum!=sum_) std::cerr << "[HM_RTK WARN] NMEA checksum mismatch: received=0x" 
                                << std::hex << sum << " calculated=0x" << sum_ << std::dec << std::endl;

        return sum == sum_;
    }

    double GGATime2Local(const std::string& time) {
        // 1. 验证输入格式
        if (!boost::regex_match(time, boost::regex("^\\d{6}\\.?\\d*$"))) {
            std::cerr << "[HM_RTK ERROR] Invalid GGA time format: " << time << std::endl;
            return -1.0;
        }
        
        // 2. 解析GGA时间字符串
        int gga_hour, gga_min, gga_sec;
        double gga_sec_fraction = 0.0;
        
        try {
            gga_hour = std::stoi(time.substr(0, 2));
            gga_min = std::stoi(time.substr(2, 2));
            gga_sec = std::stoi(time.substr(4, 2));
            
            // 验证时间范围
            if (gga_hour > 23 || gga_min > 59 || gga_sec > 59) {
                std::cerr << "[HM_RTK ERROR] Invalid GGA time values: " << gga_hour << ":" 
                        << gga_min << ":" << gga_sec << std::endl;
                return -2.0;
            }
            
            // 解析秒的小数部分
            if (time.length() > 6 && time[6] == '.') {
                gga_sec_fraction = std::stod("0" + time.substr(6));
            }
        } catch (const std::exception& e) {
            std::cerr << "[HM_RTK ERROR] GGA time parsing error: " << e.what() << std::endl;
            return -3.0;
        }
        
        // 3. 获取当前UTC时间戳和本地时区偏移
        time_t now_utc = ::time(nullptr);
        struct tm local_tm;
        localtime_r(&now_utc, &local_tm);
        int local_offset = local_tm.tm_gmtoff;  // 时区偏移秒数
        
        // 4. 获取当前UTC日期
        struct tm utc_tm;
        gmtime_r(&now_utc, &utc_tm);
        
        // 5. 构造GGA的完整UTC时间
        struct tm gga_utc_tm = utc_tm;  // 使用当前UTC日期
        gga_utc_tm.tm_hour = gga_hour;
        gga_utc_tm.tm_min = gga_min;
        gga_utc_tm.tm_sec = gga_sec;
        
        // 6. 转换为UTC时间戳
        time_t gga_utc_timestamp = timegm(&gga_utc_tm);
        if (gga_utc_timestamp == -1) {
            std::cerr << "[HM_RTK ERROR] Failed to convert GGA time to timestamp" << std::endl;
            return -4.0;
        }
        
        // 7. 处理跨日期情况
        double time_diff = difftime(gga_utc_timestamp, now_utc);
        
        if (time_diff > 12 * 3600) {
            // GGA时间比当前时间早超过12小时，可能是昨天的
            gga_utc_tm.tm_mday -= 1;
            gga_utc_timestamp = timegm(&gga_utc_tm);
            time_diff = difftime(gga_utc_timestamp, now_utc);
        } else if (time_diff < -12 * 3600) {
            // GGA时间比当前时间晚超过12小时，可能是明天的
            gga_utc_tm.tm_mday += 1;
            gga_utc_timestamp = timegm(&gga_utc_tm);
            time_diff = difftime(gga_utc_timestamp, now_utc);
        }
        
        // 8. 检查时间差是否合理（允许5分钟容差）
        if (std::abs(time_diff) > 300) {
            std::cerr << "[HM_RTK WARN] GGA time difference too large: " 
                    << time_diff << " seconds" << std::endl;
        }
        
        // 9. 转换为本地时间戳并添加小数秒
        double gga_local_timestamp = static_cast<double>(gga_utc_timestamp + local_offset) 
                                    + gga_sec_fraction;
        
        // 10. 调试输出（可选）
        // std::cout << std::fixed << std::setprecision(3)
        //         << "GGA UTC time: " << std::put_time(&gga_utc_tm, "%Y-%m-%d %H:%M:%S")
        //         << " -> Local timestamp: " << gga_local_timestamp
        //         << " (offset: " << local_offset/3600.0 << "h)" << std::endl;
        
        return gga_local_timestamp;
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
        if (nmea_split.size() != 15){
            std::cerr << "[HM_RTK ERROR] NMEA sentence split failed! Expected 15 fields, got " 
                      << nmea_split.size() << std::endl;
            return false;
        }

        static double last_gga_time = 0.0;
        double current_gga_time = std::stod(nmea_split[1]);
        if (current_gga_time == last_gga_time) {
            // std::cerr << "[HM_RTK WARN] Duplicate GGA time detected: " << current_gga_time << std::endl;
            return false; // 跳过重复的GGA
        }
        last_gga_time = current_gga_time;
        
        // 检查关键字段有效性
        auto check_field = [](const std::string& field) {
            return !field.empty() && field != "0";
        };

        if(!check_field(nmea_split[2]) || !check_field(nmea_split[4])) {
            std::cerr << "[HM_RTK ERROR] NMEA sentence invalid: " << nmea << std::endl;
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

            int satnum = strtol(nmea_split[7].c_str(), nullptr, 10);
            gnss_pos_msg.position_covariance_type = satnum;

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
                std::cerr << "[HM_RTK ERROR] NMEA sentence time parse failed! Local time=" << time_local << std::endl;
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
            gnss_pos_msg.header.frame_id = "rtk";
            // std::cout << "rtk time=" << gnss_pos_msg.header.timestamp << " cur time=" << rosTime
		    // 	<< " diff=" << rosTime - gnss_pos_msg.header.timestamp << std::endl;
        }catch(...){
            std::cerr << "[HM_RTK ERROR] NMEA sentence parse failed: " << nmea << std::endl;
            return false;
        }

        return true;
    }

    bool parse_pub_sixdof_pos(const std::string& unicore,sixdof_pose_with_cov& sixdof_pos_t)
    {
        const auto it = unicore.find(";GNSS");
        const auto it_end = unicore.find("*");
        if (it == std::string::npos && it_end ==std::string::npos){
            std::cerr << "[HM_RTK ERROR] AGRIC not found in: " << unicore << std::endl;
            return false;
        }
        std::string core_str(unicore.begin() + it,unicore.end());
        // std::cout<<"core_str "<<core_str<<std::endl;
        std::vector<std::string> unicore_split;
        boost::split(unicore_split, core_str, boost::is_any_of(","));
        if (unicore_split.size() != 57){
            std::cerr << "[HM_RTK ERROR] AGRIC Unicore sentence split failed! Expected 57 fields, got " 
                      << unicore_split.size() << std::endl;
            return false;
        }

        
        sixdof_pos_t.year       = std::strtod(unicore_split[ 2].c_str(),nullptr);
        sixdof_pos_t.month     = std::strtod(unicore_split[ 3].c_str(),nullptr);
        sixdof_pos_t.day        = std::strtod(unicore_split[ 4].c_str(),nullptr);
        sixdof_pos_t.hour       = std::strtod(unicore_split[ 5].c_str(),nullptr);
        sixdof_pos_t.minute     = std::strtod(unicore_split[ 6].c_str(),nullptr);
        sixdof_pos_t.second     = std::strtod(unicore_split[ 7].c_str(),nullptr);
        sixdof_pos_t.second_gps = std::strtod(unicore_split[47].c_str(),nullptr); /*GPS 周内毫秒*/

        sixdof_pos_t.heading_state = (int)std::strtod(unicore_split[ 9].c_str(),nullptr); /*Heading Status*/
        sixdof_pos_t.pose_type     = (int)std::strtod(unicore_split[10].c_str(),nullptr); /*流动站定位状态：*/
        sixdof_pos_t.speed_type    = (int)std::strtod(unicore_split[54].c_str(),nullptr); /*速度解状态有效*/
        sixdof_pos_t.sat           = (int)std::strtod(unicore_split[10].c_str(),nullptr); /*参与解算 GPS 卫星数*/
        sixdof_pos_t.sat          += (int)std::strtod(unicore_split[11].c_str(),nullptr); /*参与解算北斗卫星数 */
        sixdof_pos_t.sat          += (int)std::strtod(unicore_split[12].c_str(),nullptr); /*参与解算 GLONASS 卫星数 */

        

        sixdof_pos_t.ecef_x = std::strtod(unicore_split[32].c_str(),nullptr);
        sixdof_pos_t.ecef_y = std::strtod(unicore_split[33].c_str(),nullptr);
        sixdof_pos_t.ecef_z = std::strtod(unicore_split[34].c_str(),nullptr);
        sixdof_pos_t.lat    = std::strtod(unicore_split[29].c_str(),nullptr);
        sixdof_pos_t.lon    = std::strtod(unicore_split[30].c_str(),nullptr);
        sixdof_pos_t.alt    = std::strtod(unicore_split[31].c_str(),nullptr);

        double yaw     = std::strtod(unicore_split[19].c_str(),nullptr); // degree
        double pitch   = std::strtod(unicore_split[20].c_str(),nullptr);
        double roll    = std::strtod(unicore_split[21].c_str(),nullptr);
        sixdof_pos_t.q = Eigen::AngleAxisd(yaw * M_PI /180, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(pitch * M_PI /180, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(roll * M_PI /180, Eigen::Vector3d::UnitX());
        
        sixdof_pos_t.sigma_ecef_x = std::strtod(unicore_split[38].c_str(),nullptr);
        sixdof_pos_t.sigma_ecef_y = std::strtod(unicore_split[39].c_str(),nullptr);
        sixdof_pos_t.sigma_ecef_z = std::strtod(unicore_split[40].c_str(),nullptr);
        sixdof_pos_t.sigma_lat    = std::strtod(unicore_split[35].c_str(),nullptr);
        sixdof_pos_t.sigma_lon    = std::strtod(unicore_split[36].c_str(),nullptr);
        sixdof_pos_t.sigma_alt    = std::strtod(unicore_split[37].c_str(),nullptr);

        

        //东北天速度
        sixdof_pos_t.v_north   = std::strtod(unicore_split[23].c_str(),nullptr);  /*北方向速度*/
        sixdof_pos_t.v_east    = std::strtod(unicore_split[24].c_str(),nullptr);  /*东方向速度 */
        sixdof_pos_t.v_up      = std::strtod(unicore_split[25].c_str(),nullptr);  /*天顶方向速度 */

        sixdof_pos_t.sigma_vx = std::strtod(unicore_split[26].c_str(),nullptr);  /*北方向速度标准差*/
        sixdof_pos_t.sigma_vy = std::strtod(unicore_split[27].c_str(),nullptr);  /*东方向速度标准差*/
        sixdof_pos_t.sigma_vz = std::strtod(unicore_split[28].c_str(),nullptr);  /*天顶方向速度标准差*/

        

        std::cout << "[HM_RTK] " << std::setfill('0') 
                  << std::setw(4) << sixdof_pos_t.year << "-"
                  << std::setw(2) << sixdof_pos_t.month << "-" 
                  << std::setw(2) << sixdof_pos_t.day << " "
                  << std::setw(2) << sixdof_pos_t.hour << ":"
                  << std::setw(2) << sixdof_pos_t.minute << ":"
                  << std::setw(2) << sixdof_pos_t.second << "." << std::setw(3) << (sixdof_pos_t.second_gps % 1000)
                  << " State:[" << sixdof_pos_t.pose_type << "," << sixdof_pos_t.heading_state 
                  << "," << sixdof_pos_t.speed_type << "] Sat:" << sixdof_pos_t.sat
                  << " LLA:[" << std::fixed << std::setprecision(6) 
                  << sixdof_pos_t.lat << "," << sixdof_pos_t.lon << "," << sixdof_pos_t.alt
                  << "] RPY:[" << std::setprecision(2) << roll << "," << pitch << "," << yaw << "]"
                  << std::endl;

        return true;
    }

}