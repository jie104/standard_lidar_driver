//
// Created by ehl on 2023/4/25.
// Support: Hinson, LE-50821, degree range:360
//

#ifndef STANDARD_LIDAR_DRIVER_HINSON_LASER_PROTOCOL_HPP
#define STANDARD_LIDAR_DRIVER_HINSON_LASER_PROTOCOL_HPP
#include "laser_protocol_interface.hpp"
#include <fstream>
//#include ""
#include "../../core/util/time.h"
#include "../../core/util/utils.h"

namespace sros{
class HinsonLaserProtocol: public LaserProtocolInterface {
 public:
    // 雷达配置信息
    struct HinsonConfigInfo{
        uint16_t angle_range;             // 单位：度
        uint16_t angle_start_min;
        uint16_t angle_end_max;
        uint32_t total_point_numbers;     // 单圈360度 总测量点数
        uint16_t total_sections;          // 单圈360度 分几段完成测量，默认：15
        uint16_t section_frames;          // 每段测量的帧数，or：单点采样次数，默认：2
        uint32_t block_count;             // 数据块
        double angle_resolution;          // 单位：度
        double freq;                      // 扫描频率
        double period;                    // 扫描周期
        uint16_t timestamp;               // 单位 us
    };
    // 每一帧数据的结构。NOTE：雷达的电机转一圈分15次测量（sections），每一次的测量数据分2帧(frame)发送
    struct HinsonPackageBlock{
        uint16_t angle_start;             // [大端]
        uint16_t angle_end;
        uint16_t point_numbers;           // 当期数据帧的总测量点数
        uint16_t point_index;             // 当前数据帧最后检测点的顺序编号
        uint16_t section_point_numbers;   // 所有数据帧的总测量点数， 360° / N 角度范围内
        uint16_t timestamp;               // 单位 us，时间记录范围 0-65535us, 记录满后数据清零
        uint16_t block_no;                // 由计算得出：完整包的哪一块
        std::vector<uint16_t> measures;   // [小端] 测量距离的单位为 mm
        std::vector<uint16_t> intensities;
    };

    HinsonLaserProtocol(const std::string lidar_model = "HINSON_360") {
        lidar_model_ = lidar_model;
        if (lidar_model == "HINSON_360") {
            ANGLE_MIN_DEG = -180.0;
            ANGLE_MAX_DEG = 180.0;
            configInfo_.total_sections = 15;
        } else if (lidar_model == "HINSON_270") {
            ANGLE_MIN_DEG = -160.0;
            ANGLE_MAX_DEG = 160.0;
            configInfo_.total_sections = 14;
        }
        ANGLE_RANGE_DEG = ANGLE_MAX_DEG - ANGLE_MIN_DEG;
        configInfo_.angle_start_min = 0;
        configInfo_.angle_end_max = 360;
        configInfo_.angle_range = 360;
        configInfo_.freq = 20;
        configInfo_.section_frames = 1;  // 可变
        configInfo_.block_count = 0;
        last_device_timestamp = 0;
        last_sys_recv_block_time = sros::core::util::get_time_in_us();
    }

    virtual ~HinsonLaserProtocol() {
        sync_time_out.close();
    }

    virtual int findPackageStart(boost::circular_buffer<char> &ring_buffer) {
        if (ring_buffer.size() < 60) return -1;
        for (std::size_t i = 0; i < ring_buffer.size() - 6; i++) {
            if (((unsigned char) ring_buffer[i]) == 0x48 && ((unsigned char) ring_buffer[i + 1]) == 0x49 &&
                ((unsigned char) ring_buffer[i + 2]) == 0x53 && ((unsigned char) ring_buffer[i + 3]) == 0x4e) {
                return i;
            }
        }
        return -2;
    }

    virtual bool resolvePackage(boost::circular_buffer<char> &ring_buffer, int start_index) {
        // init block;
        ring_buffer.erase_begin(start_index);
        if (!parseConfig(ring_buffer)) {
            return false;
        }
        if (!parseScanPoints(ring_buffer)) {
            return false;
        }
        package_blocks.push_back(block);
        return true;
    }

    virtual bool isEndPackage() {
        auto &last_block = package_blocks.back();
        if (last_block.block_no == configInfo_.block_count) {
            return true;
        }
        return false;
    }

    virtual bool cpToScan(std::shared_ptr<ScanMsg> &scan) { // convert 2 scan
        scan.reset(new ScanMsg); // 初始化保证清空

        int64_t sync_time = getSyncTime(sys_recv_block_time, sensor_send_block_time); // use first block
        save_sync_times(sync_time);
        scan->scan_time = (double)configInfo_.period / 1e6; // 雷达转360度的时间，单位：second
#ifdef ROS_NODE
        scan->header.stamp = ros::Time::now();
#else
        scan->time_ = sync_time + 90 / 360 * scan->scan_time * 1e6; // cal middle scan time, unit us, received end scan time of first package block (6 blocks)
#endif
        int scan_point_num = configInfo_.total_point_numbers;
        scan->angle_increment = ANGLE_RANGE_DEG / scan_point_num * M_PI / 180.0;
        scan->angle_min = ANGLE_MIN_DEG * M_PI / 180.0;
        scan->angle_max = ANGLE_MAX_DEG * M_PI / 180.0;
        scan->time_increment = 1.0 / (configInfo_.freq * scan_point_num * 360 / ANGLE_RANGE_DEG);
        scan->range_min = RANGE_MIN;
        scan->range_max = RANGE_MAX;
        int total_num = 0;
        for(auto &blk : package_blocks){
            total_num += blk.point_numbers;
            for (int i = 0; i < blk.point_numbers; ++i) {
                scan->ranges.push_back(float(blk.measures[i]) * RANGE_UNIT);
//                scan->intensities.push_back(float(blk.intensities[i]) * INTENSITY_UNIT);
                scan->intensities.push_back(200);
            }
        }
        package_blocks.clear();
        return checkConvertedScan(scan, scan_point_num);
    }

    virtual bool checkScanValid() {
        if (package_blocks.size() == configInfo_.block_count) {
            if (package_blocks.back().block_no == configInfo_.block_count) {
                return true;
            }
        }
        LOG(INFO) << "package is wrong! will clear!";
        package_blocks.clear();
        return false;
    }

    virtual void getStartCmd(std::vector<std::vector<char>>& datas) {
        datas.resize(1);
        int index = 0;
        if (lidar_model_ == "HINSON_270") {
            index = 1;
        }
        for(auto& data:datas) {
            data.clear();
            data = getBody(index);
        }
    }

    std::vector<char> getBody(int head = 0){
        std::vector<char> data;
        if (head == 0) {  // LE-xxx HINSON_360
            data.push_back(0x53); // frame head
            data.push_back(0x43);
            data.push_back(0x74);
            data.push_back(0x72);
            data.push_back(0x6c);
            data.push_back(0x00); // run_state == "run"
            data.push_back(0x03); // measure_frequency_kHz == "200"
            data.push_back(0x02); // spin_frequency_Hz == "20"
            data.push_back(0x01); // sampling_size_per_position:1
            data.push_back(0x01); // noise_filter_level:1
            uint32_t wCrc = calcCrc16(data.data(), data.size());
            data.push_back(wCrc & 0x00ff);
            data.push_back((wCrc & 0xff00) >> 8);
        } else if (head == 1) {  // HE-xxx HINSON_270
            data.push_back(0x52); // frame head
            data.push_back(0x41);
            data.push_back(0x75);
            data.push_back(0x74);
            data.push_back(0x6F);
            data.push_back(0x01); // 0x01 传感器开始自动发送数据
            uint32_t wCrc = calcCrc16(data.data(), data.size());
            data.push_back(wCrc & 0x00ff);
            data.push_back((wCrc & 0xff00) >> 8);
        }
        LOG(INFO) << "cmd size " << data.size() << std::hex << " data " << numberListToStr(data.begin(), data.end());
        return data;
    }
    // get device model name string. NOTE: called before starting data stream
    virtual void packCmdGetDeviceModel(std::vector<char> &cmd) {
        std::vector<char>().swap(cmd);
        cmd.push_back(0xFF);
    }
    virtual void parseDeviceModel(std::vector<char> data, std::string &str) {
        if (!data.empty()) {

        } else {
            if (lidar_model_ == "HINSON_360") {
              str = "HINSON-LE-50821";
            } else if (lidar_model_ == "HINSON_270") {
                str = "HINSON-HE-3051";
            }
        }
    }

    // get device SN string. NOTE: called before starting data stream
    virtual void packCmdGetDeviceSn(std::vector<char> &cmd) {
        std::vector<char>().swap(cmd);
    }
    // get device SN
    virtual void parseDeviceSN(std::vector<char> data, std::string &str) {
        if (data.size() >= 12) {
            if (data[0] == 0x00 && (data[1] == 0x20) && data[6] == 0x02 && data[7] == 0x0F) {
                std::string tempstr = (char*)&data[12];
                if (tempstr.size() < 16) {
                    str = tempstr;
                } else {
                    str.assign((char*)&data[12], 16);
                }
            }
        } else {
            str = device_sn_;
        }
    }

    // get device FW version string. NOTE: called before starting data stream
    virtual void packCmdGetDeviceFwVersion(std::vector<char> &cmd) {
        std::vector<char>().swap(cmd);
    }
    // get device FW version
    virtual void parseDeviceFwVersion(std::vector<char> data, std::string &str) {
        if (data.size() >= 12) {
            if (data[0] == 0x00 && (data[1] == 0x20) && data[6] == 0x02 && data[7] == 0x0E) {
                std::string tempstr = (char*)&data[12];
                if (tempstr.size() < 16) {
                    str = tempstr;
                } else {
                    str.assign((char*)&data[12], 16);
                }
            }
        } else {
            str = device_fw_version_;
        }
    }

 private:
    bool parseConfig(boost::circular_buffer<char>& ring_buffer){
        if (ring_buffer.size() < PACKAGE_CONFIG_SIZE) {
            // LOG(ERROR) << "parse error ring_buffer: " << numberListToStr(ring_buffer.begin(), ring_buffer.end() + ring_buffer.size());
            return false;
        }
        std::vector<char> package_config_bytes(PACKAGE_CONFIG_SIZE);
        readBufferFront(ring_buffer, package_config_bytes.data(), package_config_bytes.size());
        //ring_buffer.erase_begin(package_config_bytes.size());
        cpToDataBigEndian(block.angle_start, &package_config_bytes[4], 2);
        cpToDataBigEndian(block.angle_end, &package_config_bytes[6], 2);
        cpToDataBigEndian(block.point_numbers, &package_config_bytes[8], 2);
        cpToDataBigEndian(block.point_index, &package_config_bytes[10], 2);
        cpToDataBigEndian(block.section_point_numbers, &package_config_bytes[12], 2);
        cpToDataBigEndian(block.timestamp, &package_config_bytes[14], 2);
        // LOG(INFO) << "parsed angle: " << block.angle_start << ", "<< block.angle_end \
        //  << " points:" << block.point_numbers << " section_points:" << block.section_point_numbers \
        //  << " index:" << block.point_index << " timestamp:" << block.timestamp;
        
        if (block.point_numbers && (ring_buffer.size() < PACKAGE_CONFIG_SIZE + block.point_numbers * 4)) {
            LOG(ERROR) << "!!!!parse config return, ring_buffer size:" << ring_buffer.size();
            return false;
        }
        ring_buffer.erase_begin(package_config_bytes.size());
        if (block.angle_end < block.angle_start) {
            block.angle_end += 360;
        }
        uint16_t current_section_angle_range = block.angle_end - block.angle_start;
        
        double resolution = (double)current_section_angle_range / block.section_point_numbers;
        if (!configInfo_.block_count || configInfo_.angle_resolution != resolution) {
            configInfo_.angle_resolution = resolution;
            if (block.section_point_numbers == block.point_numbers) {
                configInfo_.section_frames = 1;
            } else {
                float factor = (float)block.section_point_numbers / block.point_numbers + 0.5;
                configInfo_.section_frames = static_cast<uint16_t>(factor);
            }
            configInfo_.block_count = configInfo_.total_sections * configInfo_.section_frames;
            configInfo_.total_point_numbers = block.section_point_numbers * configInfo_.total_sections;
        }

        int average = block.section_point_numbers / configInfo_.section_frames;
        int current_section_block_no = (block.point_index + average / 2) / average; // 非均等分
        block.block_no = (block.angle_start / current_section_angle_range) * configInfo_.section_frames + current_section_block_no;
        block.angle_end = block.angle_start + (uint16_t)(block.point_index * resolution);
        if (1 != current_section_block_no) {
            block.angle_start = block.angle_start + (block.point_index - block.point_numbers) * resolution;
        }
        LOG(INFO) << "no:" << (int)block.block_no << " angle: " << block.angle_start << ", "<< block.angle_end \
         << " frames:" << configInfo_.section_frames << " total_sections:" << configInfo_.total_sections \
         << " total_points:" << configInfo_.total_point_numbers << " angle resolution:" << resolution;

        if (block.block_no == 1) {
#ifdef ROS_NODE
            sys_recv_block_time = get_time_in_us();
#else
            sys_recv_block_time = sros::core::util::get_time_in_us();
#endif
            if (last_device_timestamp >= block.timestamp) {
                sensor_send_block_time += block.timestamp + 65535;
                configInfo_.period = (double)block.timestamp + 65535 - last_device_timestamp;
            } else {
                sensor_send_block_time += block.timestamp;
                configInfo_.period = (double)block.timestamp - last_device_timestamp;
            }
            last_device_timestamp = block.timestamp;
            if (configInfo_.period < 0.1) {
                LOG(INFO) << "configInfo_.period: " << configInfo_.period;
                configInfo_.period = sys_recv_block_time - last_sys_recv_block_time;
            }
            last_sys_recv_block_time = sys_recv_block_time;
            configInfo_.freq = 1e6 / configInfo_.period;
            LOG(INFO) << "configInfo_.freq: " << configInfo_.freq;
        }
        return true;
    }

    bool parseScanPoints(boost::circular_buffer<char>& ring_buffer){
        if (ring_buffer.size() < block.point_numbers * 4) {
            LOG(ERROR) << "!!!!parse error ring_buffer size:" << ring_buffer.size();
            return false;
        }
        std::vector<char> package_data_bytes(block.point_numbers * 4);
        readBufferFront(ring_buffer, package_data_bytes.data(), package_data_bytes.size());
        ring_buffer.erase_begin(package_data_bytes.size());
        block.intensities.clear();
        block.measures.clear();
        for (int i=0; i < block.point_numbers; i++){
            block.measures.emplace_back();
            cpToDataLittleEndian(block.measures.back(), &package_data_bytes[i * 4], 2);
            block.intensities.emplace_back();
            cpToDataLittleEndian(block.intensities.back(), &package_data_bytes[i * 4 + 2], 2);
        }
        return true;
    }

    // Hinson LE-50821
    uint32_t calcCrc16(char *cBuffer, unsigned int iBufLen) {
        unsigned int i, j;
        uint32_t wCrc = 0xffff;
        uint32_t wPolynom = 0xA001;
        for (i = 0; i < iBufLen; i++) {
            wCrc ^= cBuffer[i];
            for (j = 0; j < 8; j++) {
                if (wCrc &0x0001) {
                    wCrc = (wCrc >> 1) ^ wPolynom; 
                } else {
                    wCrc = wCrc >> 1;
                }
            }
        }
        return wCrc;
    }

    bool checkXor(char *recvbuf, int recvlen) {
        if (recvlen < 4) {
            return false;
        }
        uint32_t check;
        cpToDataBigEndian(check, &recvbuf[recvlen-4], 4);
        uint32_t crc = calcCrc16(recvbuf, recvlen-4);
        if (check == crc) {
            return true;
        } else {
            LOG(ERROR) << "crc check error " << std::hex << crc << " " << check;
            return false;
        }
    }

    static bool checkConvertedScan(std::shared_ptr<ScanMsg> &scan, int scan_point_num){
        if (scan->intensities.size() < scan->ranges.size()) {
            for (int i = 0; i < scan->ranges.size() - scan->intensities.size(); ++i) {
                scan->intensities.push_back(0);
            }
        }
        if (scan->ranges.size() >= scan_point_num && scan->intensities.size() >= scan_point_num) {
            scan->ranges.resize(scan_point_num);
            scan->intensities.resize(scan_point_num);
            return true;
        } else {
            LOG(ERROR) << "check error range size:" << scan->ranges.size() << "," << scan->intensities.size();
            return false;
        }
    }

    void open_output_file() {
        sync_time_out.open("/sros/bin/sync_lidar_time.txt", std::ios::out);
        sync_time_out << "(us)system_time " << " scan_time" << " sync_time " << " circle_time" << " ratio" << std::endl;
    }

    void save_sync_times(int64_t sync_time){
        if(save_sync_time && !sync_time_out.is_open()){
            open_output_file();
        }
        if(sync_time_out.is_open()) {
            sync_time_out << sys_recv_block_time << " " << sensor_send_block_time << " " << sync_time << std::endl;
        }
    }

 private:
    std::string lidar_model_;
    float RANGE_MIN = 0.01;
    float RANGE_MAX = 40.0;
    float ANGLE_MIN_DEG = -180.0;
    float ANGLE_MAX_DEG = 180.0;
    float ANGLE_RANGE_DEG = ANGLE_MAX_DEG - ANGLE_MIN_DEG;
    const float RANGE_UNIT = 0.001; // 0.001m
    const float INTENSITY_UNIT = 1.0;
    const int PACKAGE_CONFIG_SIZE = 16;
    HinsonPackageBlock block;
    std::vector<HinsonPackageBlock> package_blocks;
    HinsonConfigInfo configInfo_;

    int64_t sys_recv_block_time;
    int64_t last_sys_recv_block_time;
    int64_t sensor_send_block_time;
    int64_t last_device_timestamp;

    std::ofstream sync_time_out;
    bool save_sync_time = false;
    std::string device_sn_ = "";
    std::string device_model_name_ = "";
    std::string device_fw_version_ = "";
};
}

#endif  // STANDARD_LIDAR_DRIVER_HINSON_LASER_PROTOCOL_HPP
