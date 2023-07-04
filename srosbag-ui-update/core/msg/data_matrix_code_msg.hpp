//
// Created by lfc on 2019/11/22.
//

#ifndef SROS_DATA_MATRIX_CODE_MSG_HPP
#define SROS_DATA_MATRIX_CODE_MSG_HPP
#include <memory>
#include <vector>
#include "base_msg.h"
#define DM_CODE_NOT_DETECTED 0
#define DM_CODE_DETECTED 1

namespace sros {
namespace core {
class DataMatrixCodeMsg : public BaseMsg {
 public:
    DataMatrixCodeMsg(std::string topic_name = "DM_CODE_INFO") : BaseMsg(topic_name, TYPE_DMCODE_DATA, true) {}

    virtual ~DataMatrixCodeMsg() {}

    uint64_t getTimestamp() const { return (uint64_t)time_; }

    void setTimestamp(const uint64_t timestamp) { time_ = timestamp; }

    virtual void getTime(){

    }

    void setCameraName(const std::string& name) { camera_name_ = name; }

    const std::string& getCameraName() const { return camera_name_; }

    void setCodeInfo(const int state, const int x, const int y, const int angle, const int code_int) {
        state_ = state;
        const double tithe_mm_to_m = 0.0001;
        const double milli_rad_to_rad = 0.001;
        x_ = (double)x * tithe_mm_to_m;
        y_ = (double)y * tithe_mm_to_m;
        angle_ = (double)angle * milli_rad_to_rad;
        code_int_ = code_int;
    }

    void getCodeInfo(int& state, int& x, int& y, int& angle, int& code_int) const {
        state = state_;
        x = x_;
        y = y_;
        angle = angle_;
        code_int = code_int_;
    }

    const std::string& getCodeStr() const { return code_str_; }

    void setCodeStr(const std::string& code_str) { code_str_ = code_str; }

    int state_ = DM_CODE_NOT_DETECTED;  //识别到为1,否则为0
    double x_ = 0;                         //单位为m,有效值0.0001m
    double y_ = 0;                         //单位为m,有效值0.0001m
    double angle_ = 0;                     //单位为rad,,有效值0.0001rad
    int code_int_ = 0;                  //从二维码信息中提取出来的int值
    std::string code_str_;              //完整的二维码编码信息
    std::string camera_name_;           //摄像头信息
};
typedef std::shared_ptr<DataMatrixCodeMsg> DataMatrixCodeMsg_ptr;
}  // namespace core
}  // namespace sros

#endif  // SROS_DATA_MATRIX_CODE_MSG_HPP
