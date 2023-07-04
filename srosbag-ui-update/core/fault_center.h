/**
 * @file fault_center
 *
 * @author pengjiali
 * @date 20-6-3.
 *
 * @describe
 *
 * @copyright Copyright (c) 2020 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_FAULT_CENTER_H
#define SROS_FAULT_CENTER_H

#include <functional>
#include <list>
#include <map>
#include "device/device.h"
#include "fault_code.h"

namespace sros {
namespace core {

enum FaultLevel {
    INFO = 1,
    WARNING = 2,
    ERROR = 3,
    FATAL = 4,
};

class Fault {
 public:
    sros::device::DeviceID getDevice() { return device_id; }
    friend std::ostream& operator<<(std::ostream& out, const Fault &fault) {
        out << "Fault{id: " << fault.id << ", level: " << fault.level << ", name: " << fault.name
            << ", response_behavior: " << fault.response_behavior << "}";
        return out;
    }

    FaultCode id = FAULT_CODE_NONE;
    sros::device::DeviceID device_id = sros::device::DEVICE_ID_UNDEFINED;
    std::string device_name;
    std::string name;
    std::string describe;
    std::string how_to_fix;
    FaultLevel level = FATAL;
    uint32_t raise_timestamp = 0;  // 触发的时间
    int music = 0;
    int response_behavior = FAULT_RESPONSE_BEHAVIOR_NONE;
    int priority = 0;                                     // 优先级，值越大优先级越高
    std::function<void()> try_to_recover_func = nullptr;  // 尝试恢复的函数
    std::function<bool()> can_automatically_recover_func = []() {
        return false;
    };  // 是否可以尝试自动恢复，若能则可以调用try_to_recover_func
};

using Fault_Ptr = std::shared_ptr<Fault>;

class FaultCenter {
 public:
    static FaultCenter* getInstance();

    void track(FaultCode fault_code, std::function<bool(void)> trigger_func,
               std::function<bool()> can_automatically_recover_func = nullptr,
               std::function<void()> try_to_recover_func = nullptr);

    void addFault(int fault_code, std::function<bool()> can_automatically_recover_func = nullptr,
                  std::function<void()> try_to_recover_func = nullptr);
    void removeFault(int fault_code);

    std::list<Fault_Ptr> getFaultList() const;
    Fault getFirstFault() const;  // 获取第一个故障，也是获取最严重的故障

    void recover(int fault_code);  // 尝试恢复某个故障

    Fault_Ptr truckMovementTaskRunningFault() const;  // 获取卡住移动任务不能运行的故障
    Fault_Ptr truckActionTaskRunningFault() const;    // 获取卡住动作任务不能运行的故障

    void setMusicId(FaultCode fault_id, int music_id); // 设置故障的音乐
 private:
    FaultCenter();
    void loadAllFault();
    void loadDefaultFaultHandler(Fault_Ptr fault_ptr);
    /**
     * 用户可以通过参数"inspection.disable_" + 设备名是否忽略这一类设备；
     * @param fault_ptr
     */
    void loadDisableInspectionFault(Fault_Ptr fault_ptr);
    void handleFault(Fault_Ptr fault_ptr);

    mutable boost::shared_mutex mutex_;  // 这个锁专门锁Fault_list
    std::list<Fault_Ptr> fault_list;

    std::map<FaultCode, Fault_Ptr> fault_map;
};
}  // namespace core
}  // namespace sros
#endif  // SROS_FAULT_CENTER_H
