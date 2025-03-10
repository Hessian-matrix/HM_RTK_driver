#include "HM_RTK/utils.hpp"
#include <vector>
#include <chrono>
#include <boost/algorithm/string.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/asio.hpp>
#include <boost/regex.hpp>
#include <std_msgs/Header.h>


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

        if(sum!=sum_) printf("%02x %02x %s\n",sum, sum_, nmea_sum);

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
            return -1.0;
        }
        time_t gga_time = mktime(&gga_tm) + local_offset;
        double gga_time_local = gga_time + seconds;
    
        // Calculate the difference between the current time and the GGA time
        double time_difference = std::difftime(gga_time, now_time);
        // std::cout << std::setprecision(16) << "gga time=" << gga_time_local << " now=" << now_time << ", timezone_offset=" << local_offset << ", diff=" << time_difference << " str=" << time << std::endl; 
    
        // Ensure the time difference is not excessively large
        if (std::abs(time_difference) > 60) {
            return -1.0; // Invalid time
        }
    
        // 获取gga_time的UTC时间戳，也就是1970年1月1日0时0分0秒到gga_time的秒数
        return gga_time_local;
    }

    /// @brief 解析GGA字符串，获得经纬度给ntrip client
    /// @param nmea GGA字符串
    /// @param lon_nmea 
    /// @param lat_nmea 
    /// @return 
    bool parse_pub_nmea(const std::string& nmea, sensor_msgs::NavSatFix& gnss_pos_msg)
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

            sensor_msgs::NavSatStatus gnss_pos_status;
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
                std::cerr << "NMEA Sentence Time Parse Failed!" << std::endl;
                return false;
            }
            gnss_pos_msg.header.stamp.fromSec(time_local);
            gnss_pos_msg.header.frame_id = "rtk_link";
            // std::cout << "rtk time=" << gnss_pos_msg.header.timestamp << " cur time=" << rosTime
		    // 	<< " diff=" << rosTime - gnss_pos_msg.header.timestamp << std::endl;
        }catch(...){
            std::cerr << "NMEA Sentence Parse Failed!" << std::endl;
            return false;
        }

        return true;
    }
}