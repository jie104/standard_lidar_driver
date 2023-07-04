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

#include "fault_center.h"
#include <math.h>
#include <algorithm>
#include "core/settings.h"
#include "db/db.h"
#include "src.h"
#include "core/msg/command_msg.hpp"
#include "core/msg_bus.h"

namespace sros {
namespace core {

FaultCenter *FaultCenter::getInstance() {
    static FaultCenter instance;
    return &instance;
}

FaultCenter::FaultCenter() { loadAllFault(); }

void FaultCenter::track(FaultCode fault_code, std::function<bool(void)> trigger_func,
                        std::function<bool()> can_automatically_recover_func,
                        std::function<void()> try_to_recover_func) {
    bool trigger = trigger_func();
    if (trigger) {
        addFault(fault_code, can_automatically_recover_func, try_to_recover_func);
    } else {
        removeFault(fault_code);
    }
}

void FaultCenter::addFault(int fault_code, std::function<bool()> can_automatically_recover_func,
                           std::function<void()> try_to_recover_func) {
    boost::lock_guard<boost::shared_mutex> lock(mutex_);
    for (auto fault : fault_list) {
        if (fault_code == fault->id) {
            return;
        }
    }

    try {
        auto fault = fault_map.at((FaultCode)fault_code);
        if (can_automatically_recover_func != nullptr) {
            fault->can_automatically_recover_func = can_automatically_recover_func;
        }
        if (try_to_recover_func != nullptr) {
            fault->try_to_recover_func = try_to_recover_func;
        }
        fault->raise_timestamp = sros::core::util::get_timestamp_in_s();
        fault_list.push_back(fault);
        fault_list.sort([](Fault_Ptr &a, Fault_Ptr &b) {
            if (a->priority == b->priority) {
                if (a->response_behavior == b->response_behavior) {
                    return a->level > b->level;
                } else {
                    return a->response_behavior > b->response_behavior;
                }
            }
            return a->priority > b->priority;
        });
        handleFault(fault);
    } catch (const std::exception &e) {
        LOG(ERROR) << e.what();
    }
}

void FaultCenter::removeFault(int fault_code) {
    boost::lock_guard<boost::shared_mutex> lock(mutex_);
    for (auto it = fault_list.cbegin(); it != fault_list.cend(); ++it) {
        if (fault_code == (*it)->id) {
            auto fault = *it;
            fault_list.erase(it);

            handleFault(fault);
            break;
        }
    }
}

std::list<Fault_Ptr> FaultCenter::getFaultList() const {
    boost::shared_lock<boost::shared_mutex> lock(mutex_);
    return fault_list;
}

Fault FaultCenter::getFirstFault() const {
    boost::shared_lock<boost::shared_mutex> lock(mutex_);
    if (fault_list.empty()) {
        return Fault();
    }
    return *fault_list.front().get();
}

Fault_Ptr FaultCenter::truckMovementTaskRunningFault() const {
    boost::shared_lock<boost::shared_mutex> lock(mutex_);
    for (auto fault : fault_list) {
        if (fault->response_behavior == FAULT_RESPONSE_BEHAVIOR_MOVEMENT_DISABLE_PAUSE_MANUAL_CONTINUE ||
            fault->response_behavior == FAULT_RESPONSE_BEHAVIOR_MOVEMENT_DISABLE_PAUSE_AUTO_CONTINUE ||
            fault->response_behavior == FAULT_RESPONSE_BEHAVIOR_ALL_DISABLE_PAUSE_MANUAL_CONTINUE ||
            fault->response_behavior == FAULT_RESPONSE_BEHAVIOR_ALL_DISABLE_PAUSE_AUTO_CONTINUE) {
            return fault;
        }
    }
    return nullptr;
}

Fault_Ptr FaultCenter::truckActionTaskRunningFault() const {
    boost::shared_lock<boost::shared_mutex> lock(mutex_);
    for (auto fault : fault_list) {
        if (fault->response_behavior == FAULT_RESPONSE_BEHAVIOR_ACTION_DISABLE_PAUSE_MANUAL_CONTINUE ||
            fault->response_behavior == FAULT_RESPONSE_BEHAVIOR_ACTION_DISABLE_PAUSE_AUTO_CONTINUE ||
            fault->response_behavior == FAULT_RESPONSE_BEHAVIOR_ALL_DISABLE_PAUSE_MANUAL_CONTINUE ||
            fault->response_behavior == FAULT_RESPONSE_BEHAVIOR_ALL_DISABLE_PAUSE_AUTO_CONTINUE) {
            return fault;
        }
    }
    return nullptr;
}

void FaultCenter::recover(int fault_code) {
    boost::shared_lock<boost::shared_mutex> lock(mutex_);
    for (auto fault : fault_list) {
        if (fault_code == fault->id) {
            if (fault->can_automatically_recover_func()) {
                fault->try_to_recover_func();
            } else {
                LOG(INFO) << "fault " << fault_code << " can not recover!";
            }
            return;
        }
    }

    LOG(INFO) << "fault " << fault_code << " not found!";
}

void FaultCenter::loadAllFault() {
    const std::string sql =
        "SELECT fault_code.id, device_id, device_name, level, name, what, how_to_fix, music_id, response_behavior, "
        "priority FROM fault_code INNER JOIN device_tree ON fault_code.device_id = device_tree.id;";

    try {
        std::lock_guard<std::recursive_mutex> db_mutex(g_db_mutex);
        SQLite::Statement query(g_db, sql);
        while (query.executeStep()) {
            auto fault = std::make_shared<Fault>();
            fault->id = (FaultCode)query.getColumn(0).getInt();
            fault->device_id = (sros::device::DeviceID)query.getColumn(1).getInt();
            fault->device_name = query.getColumn(2).getString();
            fault->level = (FaultLevel)query.getColumn(3).getInt();
            fault->name = query.getColumn(4).getString();
            fault->describe = query.getColumn(5).getString();
            fault->how_to_fix = query.getColumn(6).getString();
            fault->music = query.getColumn(7).getInt();
            fault->response_behavior = query.getColumn(8).getInt();
            fault->priority = query.getColumn(9).getInt();
            loadDefaultFaultHandler(fault);
            loadDisableInspectionFault(fault);
            fault_map.insert(std::make_pair(fault->id, fault));
        }
    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a exception: " << e.what() << "; sql is " << sql;
    }
}
void FaultCenter::loadDefaultFaultHandler(Fault_Ptr fault_ptr) {
    switch (fault_ptr->device_id) {
        case sros::device::DEVICE_ID_MC_MOTOR_1:
        case sros::device::DEVICE_ID_MC_MOTOR_2:
        case sros::device::DEVICE_ID_AC_MOTOR_1:
        case sros::device::DEVICE_ID_AC_MOTOR_2: {
            fault_ptr->can_automatically_recover_func = [&]() {
                return !g_state.isEmergency() && !g_state.isBreakSwitchON() &&
                       !g_state.isPowerSaveMode();  // 电机异常只有在上完点之后才好使
            };
            fault_ptr->try_to_recover_func = [&]() { src_sdk->resetFault(); };
            break;
        }
        case sros::device::DEVICE_ID_EAC: {
            if (fault_ptr->id == FAULT_CODE_EAC_UNKNOWN_FAULT || fault_ptr->id == FAULT_CODE_EAC_OTHER_FAULT) {
                fault_ptr->can_automatically_recover_func = [&]() {
                  return true;
                };
                fault_ptr->try_to_recover_func = [&]() {
                  auto d_msg = std::make_shared<sros::core::CommandMsg>("fault_center");
                  d_msg->command = sros::core::CMD_RESET_FAULT;
                  sros::core::MsgBus::sendMsg(d_msg);
                };
            }
            break;
        }
        default: {
            break;
        }
    }
}

void FaultCenter::loadDisableInspectionFault(Fault_Ptr fault_ptr) {
    auto &s = sros::core::Settings::getInstance();
    bool disable_inspection =
        (s.getValue<std::string>("inspection.disable_" + fault_ptr->device_name, "False") == "True");
    if (disable_inspection) {
        fault_ptr->response_behavior = FAULT_RESPONSE_BEHAVIOR_NONE;
    }
}

void FaultCenter::handleFault(Fault_Ptr fault_ptr) {
    if (fault_ptr->level < ERROR || fault_ptr->response_behavior == FAULT_RESPONSE_BEHAVIOR_NONE) {
        return;  // 该故障不会有任何响应
    }

    bool pause_movement = false;
    bool pause_movement_is_latch = false;  // 是否锁存
    for (auto fault : fault_list) {
        LOG(INFO) << *fault;
        if (fault->level > WARNING) {
            if (fault->response_behavior == FAULT_RESPONSE_BEHAVIOR_MOVEMENT_DISABLE_PAUSE_MANUAL_CONTINUE ||
                fault->response_behavior == FAULT_RESPONSE_BEHAVIOR_ALL_DISABLE_PAUSE_MANUAL_CONTINUE) {
                pause_movement = true;
                pause_movement_is_latch = true;
            } else if (fault->response_behavior == FAULT_RESPONSE_BEHAVIOR_MOVEMENT_DISABLE_PAUSE_AUTO_CONTINUE ||
                       fault->response_behavior == FAULT_RESPONSE_BEHAVIOR_ALL_DISABLE_PAUSE_AUTO_CONTINUE) {
                pause_movement = true;
            } else if (fault->response_behavior == FAULT_RESPONSE_BEHAVIOR_EMERGENCY) {
                auto d_msg = std::make_shared<sros::core::CommandMsg>("fault_center");
                d_msg->command = sros::core::CMD_TRIGGER_EMERGENCY;
                sros::core::MsgBus::sendMsg(d_msg);
            }
        }
    }

    src_car.setPauseState(PAUSE_SOURCE_FAULT, pause_movement, SRC_MIN_PAUSE_LEVEL, pause_movement_is_latch);
}

void FaultCenter::setMusicId(FaultCode fault_id, int music_id) {
    try {
        fault_map[fault_id]->music = music_id;
    } catch (const std::exception &e) {
        LOG(ERROR) << e.what();
    }
}
}  // namespace core
}  // namespace sros
