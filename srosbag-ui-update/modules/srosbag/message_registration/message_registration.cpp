#include "factory/export_macro.hpp"
#include "../../../../core/msg/odometry_msg.hpp"

#include "../../../../core/msg/image_msg.hpp"
//EXPORT_MSG("imu", Imu)

//EXPORT_MSG(nav_msgs::Odometry::msgType(), nav_msgs::Odometry)
//EXPORT_MSG("odometryBag",sros::core::nav_msgs::Odometry)
//EXPORT_MSG(camera_msg::Camera::msgType(), camera_msg::Camera)

EXPORT_MSG("TOPIC_LASER",sros::core::LaserScanMsg )
EXPORT_MSG("FIRST_SCAN",sros::core::LaserScanMsg )
EXPORT_MSG("SECOND_SCAN",sros::core::LaserScanMsg )

//EXPORT_MSG("FIRST_SCAN", sros::core::LaserScanMsg)
//EXPORT_MSG("SECOND_SCAN", sros::core::LaserScanMsg)

//EXPORT_MSG("TOPIC_COLOR", sros::core::ImageMsg,false)

//EXPORT_MSG("TOPIC_D435_DEPTH", sros::core::ImageMsg,true)


EXPORT_MSG("LIVOX_POINTS", sros::core::LivoxPointsMsg)

EXPORT_MSG("LIVOX_IMU", sros::core::ImuMsg)

EXPORT_MSG("OdoPoseStamped", sros::core::PoseStampedMsg)

EXPORT_MSG("TOPIC_MATCHPOSE", sros::core::PoseStampedMsg)