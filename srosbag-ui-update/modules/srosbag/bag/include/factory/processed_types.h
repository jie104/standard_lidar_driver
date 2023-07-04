/**
 * @file msg_config.h
 * @author zmy (626670628@qq.com)
 * @brief 用于设置哪些message可以record 和 play
 * @version 0.1
 * @date 2021-05-17
 * 
 * 
 */

#ifndef PROCESSED_TYPES_H
#define PROCESSED_TYPES_H

#include "type_container.hpp"

#include "../../message/imu_msg.hpp"
#include "../../message/odometry_msg.hpp"

#include "../../message/lidar_msg.hpp"
#include "../../../../core/msg/laser_scan_msg.hpp"
#include "../../../../core/msg/image_msg.hpp"
#include "../../../../core/msg/livox_points_msg.hpp"
#include "../../../../core/msg/imu_msg.hpp"
#include "../../../../core/msg/PoseStampedMsg.h"

HANDLE_TYPES(
    //Imu,
   // sros::core::nav_msgs::Odometry,
    sros::core::LaserScanMsg,
//    sros::core::ImageMsg,
    sros::core::LivoxPointsMsg,
    sros::core::PoseStampedMsg,
    sros::core::ImuMsg
    )

#endif
