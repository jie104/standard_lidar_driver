//
// Created by lhx on 15-12-23.
//

#ifndef SROS_COMMON_STATE_MSG_H
#define SROS_COMMON_STATE_MSG_H

#include "base_msg.h"
#include "../pose.h"
#include <memory>
#include <vector>

namespace sros {
namespace core {

template <typename T>
class CommonStateMsg : public BaseMsg {
public:
    CommonStateMsg(topic_t topic) : BaseMsg(topic, TYPE_COMMON_STATE){ };
    virtual ~CommonStateMsg() {};

    virtual void getTime() {};

    T state;

    int param_int;

    std::string map_name;

    int failed_code_; // 失败代码
};

}
}


#endif //SROS_COMMON_STATE_MSG_H
