//
// Created by lfc on 2023/2/16.
//

#ifndef STANDARD_LIDAR_DRIVER_FREE_OPTICS_LASER_PROTOCOL_HPP
#define STANDARD_LIDAR_DRIVER_FREE_OPTICS_LASER_PROTOCOL_HPP

#include <glog/logging.h>
#include "laser_protocol_interface.hpp"
#include "synchronize_time_manager.hpp"

namespace sros {
class FreeOpticsLaserProtocol : public LaserProtocolInterface {
 public:
    struct FreePackageData {
        uint16_t check_sum;
        uint16_t cmd_type = 0x01;
        uint16_t total_index;
        uint8_t sub_index;
        uint16_t frequency;
        uint16_t angle_increment;
        uint16_t total_point_num;
        uint16_t first_point_index;
        uint16_t package_point_num;
        uint8_t second;
        uint32_t ns;
        uint64_t stamp;
        std::vector<uint16_t> measures;
        std::vector<uint16_t> intensities;
    };

    struct FreeScanData {
        LaserConfig config;
        std::vector<std::shared_ptr<FreePackageData>> package_datas;
        int64_t end_scan_time;
        int64_t end_package_time;
    };

    FreeOpticsLaserProtocol(const std::string lidar_model) {
        if (lidar_model == "h100")
            angle_range_ = 270;
        else
            angle_range_ = 360;
        scan_data_.config.angle_min = -angle_range_ / 2 / 180.0 * M_PI;
        scan_data_.config.angle_max = angle_range_ / 2 / 180.0 * M_PI;
        scan_data_.config.angle_increment = 0.1 / 180.0 * M_PI;
        scan_data_.config.time_increment = 1.0 / 30 / (angle_range_ / 0.1);
        scan_data_.config.range_min = 0.05;
        scan_data_.config.range_max = 30.0;
        scan_data_.config.scan_size = static_cast<int>(angle_range_ / 0.1); //采样点个数
        scan_data_.config.half_circle_time_in_us = 1.0 / 30.0 * 1.0e6 / 2;  // us
    }

    virtual ~FreeOpticsLaserProtocol() {}

    virtual int findPackageStart(boost::circular_buffer<char> &ring_buffer) {
        if (ring_buffer.size() < 552) return -1;
        LOG(INFO) << "ring_buffer: " << ring_buffer.size();
        for (std::size_t i = 0; i < ring_buffer.size() - 4; i++) {
            if (((unsigned char) ring_buffer[i]) == 0x02 && ((unsigned char) ring_buffer[i + 1]) == 0x02 &&
                ((unsigned char) ring_buffer[i + 2]) == 0x02 && ((unsigned char) ring_buffer[i + 3]) == 0x02) {
                return i;
            }
        }
        return -2;
    }

    virtual bool resolvePackage(boost::circular_buffer<char> &ring_buffer, int start_index) {
        int16_t data_length;
        const int header_size = 2;
        if (ring_buffer.size() >= start_index + 10) {
//            LOG(INFO) << "ring_buffer.size(): " << ring_buffer.size();
            if (ring_buffer[start_index + 6] == 0x10) {
                LOG(INFO) << "get response cmd! will transmit the laser !";
                ring_buffer.erase_begin(start_index + 4);
                return false;
            }else if(ring_buffer[start_index + 6] != 0x12){
                LOG(INFO) << "other cmd! will return false;";
                ring_buffer.erase_begin(start_index + 4);
                return false;
            }else if(ring_buffer[start_index + 6] == 0x12){
                if (ring_buffer[start_index + 7] == 0x31&&ring_buffer[start_index + 8] == 0x01) {
                    LOG(INFO) << "get response cmd! will transmit the laser! ";
                    ring_buffer.erase_begin(start_index + 4);
                    return false;
                }
            }
        }
        cpToDataBigEndian(data_length, &ring_buffer[start_index + 4], header_size);
        if (ring_buffer.size() >= data_length + start_index) {
            ring_buffer.erase_begin(start_index);
        }else{
            LOG(INFO) << "cannot get whole frame!";
            return false;
        }
        package_datas_.resize(data_length);
        readBufferFront(ring_buffer, package_datas_.data(), data_length);
        ring_buffer.erase_begin(data_length);
        bool isFirstPackage = false;
        bool isPackage = false;
        LOG(INFO) << "sub_index: " << (int)package_datas_[11];
        if (package_datas_[7] == 0x32) {
            isPackage = true;
            if (package_datas_[11] == 0x00) {
                isFirstPackage = true;
            }
        }

        if (isPackage) {
            std::shared_ptr<FreePackageData> scan_ptr(new FreePackageData);
            auto &scan_data = *scan_ptr;
            if (checkSum(package_datas_.data(), data_length - 1, package_datas_[data_length - 1])) {
                cpToDataBigEndian(scan_data.total_index, &package_datas_[9], 2);
                cpToDataBigEndian(scan_data.sub_index, &package_datas_[11], 1);
                cpToDataBigEndian(scan_data.frequency, &package_datas_[12], 2);
                cpToDataBigEndian(scan_data.angle_increment, &package_datas_[14], 2);
                cpToDataBigEndian(scan_data.total_point_num, &package_datas_[16], 2);
                cpToDataBigEndian(scan_data.first_point_index, &package_datas_[18], 2);
                cpToDataBigEndian(scan_data.package_point_num, &package_datas_[20], 2);

                cpToDataBigEndian(scan_data.second, &package_datas_[data_length - 9], 4);
                cpToDataBigEndian(scan_data.ns, &package_datas_[data_length - 5], 4);
                scan_data.ns = (uint32_t)round((double)scan_data.ns / pow(2, 32) * pow(10, 9));
                scan_data.stamp = ((int64_t)scan_data.second * 1e9 + (int64_t)scan_data.ns) / 1e3;
                scan_data_.end_package_time = scan_data.stamp;
                if (isFirstPackage) {
                    scan_data_.package_datas.clear();
                }
                if (scan_data.sub_index == 0x00) {
#ifdef ROS_NODE

                    scan_data_.end_scan_time = get_time_in_us();
#else
                    scan_data_.end_scan_time = sros::core::util::get_time_in_us();
#endif
                    //                    cpToDataBigEndian(scan_data.stamp, &package_datas_[15 + 4 *
                    //                    scan_data.package_point_num], 10); scan_data_.end_package_time =
                    //                    static_cast<int64_t> (scan_data.stamp * 1e3);
                }
                //科力雷达的时间戳是每圈第一个点在雷达时间轴上的发生时间
                scan_data.measures.clear();
                scan_data.intensities.clear();
                for (int i = 0; i < scan_data.package_point_num; ++i) {
                    scan_data.measures.emplace_back();
                    cpToDataBigEndian(scan_data.measures.back(), &package_datas_[i * 4 + 22], 2);
                    scan_data.intensities.emplace_back();
                    cpToDataBigEndian(scan_data.intensities.back(), &package_datas_[i * 4 + 24], 2);
                }
                scan_data_.package_datas.push_back(scan_ptr);
                return true;
            }
        }
        return false;
    }

    virtual void getStartCmd(std::vector<char> &data) {
        data.clear();
        data.push_back(0x02);
        data.push_back(0x02);
        data.push_back(0x02);
        data.push_back(0x02);
        data.push_back(0x00);
        data.push_back(0x0A);
        data.push_back(0x02);
        data.push_back(0x31);
        data.push_back(0x01);
        data.push_back(0x46);
    }

    virtual void getStartCmd(std::vector<std::vector<char>> &data) {
        data.emplace_back();
        getStartCmd(data.back());
    }

    template<class Type>
    void cpToData(Type &value, char *data, int size) {
        //科力的大部分数据是小端模式，低字节在低位
        value = (Type) 0;
        for (int i = 0; i <= size - 1; ++i) {
            value |= (unsigned char) data[i] << (i * 8);
        }
    }

    template<class Type>
    void cpToDataBigEndian(Type &value, char *data, int size) {
        //大端模式
        value = (Type) 0;
        for (int i = size - 1; i >= 0; --i) {
            value |= (unsigned char) data[size - 1 - i] << (i * 8);
        }
    }

    virtual bool isEndPackage() {
        auto &last_package = scan_data_.package_datas.back();
        if (last_package->first_point_index + last_package->package_point_num == last_package->total_point_num) {
            return true;
        }
        if(scan_data_.package_datas.size() > scan_data_.package_datas.back()->sub_index + 1){//TODO
            LOG(INFO) << "size: " << scan_data_.package_datas.size();
            scan_data_.package_datas.clear();
        }

        return false;
    }

    virtual bool cpToScan(std::shared_ptr<ScanMsg> &scan) {
        if (!scan) {
            scan.reset(new ScanMsg);
        }
        initializeScan(scan_data_, scan);
        auto &ranges = scan->ranges;
        auto &intens = scan->intensities;
        ranges.clear();
        intens.clear();
        for (auto &package: scan_data_.package_datas) {
            //int i = 0;
                for (auto &measure: package->measures) {
                    ranges.push_back(float(measure) / 1000.0f);
                    //i++;
                    //LOG(INFO) << "!!!!!keli - ranges " << i  << ": " << measure;
                }
                //LOG(INFO) << "!!!!!keli - ranges package_no" << package->package_no  << ", " << i;
                for (auto &inten: package->intensities) {
                    intens.push_back(float(inten));
                    //i++;
                    //LOG(INFO) << "!!!!!keli - intens " << i << ": " << measure;
                }
        }
        if (intens.size() < ranges.size()) {
            int delta = ranges.size() - intens.size();
            for (int i = 0; i < delta; ++i) {
                intens.push_back(0);
            }
        }
        scan_data_.package_datas.clear();
        if (ranges.size() >= scan_data_.config.scan_size && intens.size() >= scan_data_.config.scan_size) {
            ranges.resize(scan_data_.config.scan_size);
            intens.resize(scan_data_.config.scan_size);
            return true;
        } else {
            LOG(INFO) << "range size:" << ranges.size() << "," << intens.size();
        }
        return true;
    }

    virtual bool checkScanValid() {
        if (scan_data_.package_datas.size() == scan_data_.package_datas.back()->sub_index + 1) {
            return true;
        }
        scan_data_.package_datas.clear();   //TODO

        return false;
    }

    bool checkSum(char *recvbuf, int recvlen, uint8_t check_sum) {
        uint8_t sum = 0;
        for (auto i = 0; i < recvlen; i++) {
            sum += (unsigned char) *(recvbuf + i);
        }
        if (check_sum == sum) {
            return true;
        } else
            return false;
    }

 private:
    void initializeScan(FreeScanData &scan_data, std::shared_ptr<ScanMsg> &scan) {
        int64_t sync_time = getSyncTime(scan_data.end_scan_time, scan_data.end_package_time);
        auto &config = scan_data.config;
        scan->scan_time = 2.0 * config.half_circle_time_in_us;
#ifdef ROS_NODE
        scan->header.frame_id = "laser";
        scan->header.stamp = ros::Time::now();
#else
        scan->time_ = sync_time + config.half_circle_time_in_us;    //科力雷达0°在正前方，时间戳反映0°时刻
#endif
        scan->angle_increment = config.angle_increment;
        scan->angle_min = config.angle_min;
        scan->angle_max = config.angle_max;
        scan->time_increment = config.time_increment;
        scan->range_min = config.range_min;
        scan->range_max = config.range_max;
    }

    FreeScanData scan_data_;
    std::vector<char> package_datas_;
    double angle_range_;
};

}  // namespace sros


#endif  // STANDARD_LIDAR_DRIVER_FREE_OPTICS_LASER_PROTOCOL_HPP
