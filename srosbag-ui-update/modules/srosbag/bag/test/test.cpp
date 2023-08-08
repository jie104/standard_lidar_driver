// #include "type_container.hpp"
#include <future>
#include <QDialog>
#include "qt/progressbar.h"
#include "qt/playbag.h"
#include "qt/qtshow.h"
#include <QTimer>
#include <QLabel>
#include <QApplication>
#include <QProgressDialog>
#include <mutex>
#include <array>
#include <functional>
#include <iostream>
#include <map>
#include <typeinfo>
#include <vector>

#include <ros/ros.h>
#include <ros/time.h>
#include <rosbag/bag.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <image_transport/image_transport.h>   //image_transport
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <livox_CustomMsg/CustomMsg.h>
// #include <livox_CustomMsg/CustomPoint.h>
//#include "/home/dzl/tsdf_salm_ws/devel/include/tsdf_slam/CustomMsg.h"
//#include "/home/dzl/tsdf_salm_ws/devel/include/tsdf_slam/CustomPoint.h"

#include "bag/message_bag.h"
#include "factory/my_factory.h"
#include "../../message/lidar_msg.hpp"
#include "../../../../core/msg/PoseStampedMsg.h"
#include "../../../../core/msg/livox_points_msg.hpp"
#include "../../../../core/msg/imu_msg.hpp"

using namespace std;

/*!
 *\brief Livox lidar data type
 *  Added tag, line and offset time data on the general pcl::PointCloudXYZI data format
 */
struct LivoxPointType {
  PCL_ADD_POINT4D

  PCL_ADD_INTENSITY;
  uint8_t tag;
  uint8_t line;
  uint32_t offset_time;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (LivoxPointType,
                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                       (uint8_t, tag, tag)(uint8_t, line, line)(uint32_t, offset_time,
                                                                                offset_time)
)

int main(int argc, char **argv) {
  auto bag = bag::MsgBag::Create("/home/zxj/data/sbag/");
  //! Ros initialization
  ros::init(argc, argv, "laser_publish");
  ros::NodeHandle nd;
//  ros::Publisher livox_pc2 = nd.advertise<sensor_msgs::PointCloud2>("livox/lidar_pc2", 100);
//  ros::Publisher livox_customMsg = nd.advertise<tsdf_slam::CustomMsg>("livox/lidar", 100);
  ros::Publisher imu = nd.advertise<sensor_msgs::Imu>("imu", 500);
  ros::Publisher poseStampedMsg = nd.advertise<geometry_msgs::PoseStamped>("pose/stamped", 500);
  ros::Publisher laser = nd.advertise<sensor_msgs::LaserScan>("laser", 100);
  //! Creat ros bag file
  rosbag::Bag ros_bag;
  ros_bag.open("/home/zxj/data/sbag/output.bag", rosbag::bagmode::Write);

  // sensor_msgs PointCloud2
//    bag->setMsgHandle<sros::core::LivoxPointsMsg>
//            ([&livox_pc2, &ros_bag](const sros::core::LivoxPointsMsg &livoxIn) {
////                 std::cout << "现在还在转换，时间会很长，请稍等11111111！！" << std::endl;
//                 sensor_msgs::PointCloud2 pointOut;
//                 pcl::PointCloud<LivoxPointType>::Ptr pc2(new pcl::PointCloud<LivoxPointType>);
////                 pcl::PointCloud<pcl::PointXYZI>::Ptr pc2(new pcl::PointCloud<pcl::PointXYZI>);
//
//                 if (livoxIn.time_ > 0 && livoxIn.time_ < 1e16 && !livoxIn.points.empty()) {
////                 LOG(INFO) << livoxIn.topic_ << ", " << livoxIn.time_;
//                     pc2->width = livoxIn.points.size();
//                     pc2->height = 1;
//                     pc2->points.resize(livoxIn.points.size());
//                     for (size_t i = 0; i < livoxIn.points.size(); i++) {
//                         pc2->points[i].x = livoxIn.points[i].x;
//                         pc2->points[i].y = livoxIn.points[i].y;
//                         pc2->points[i].z = livoxIn.points[i].z;
//                         pc2->points[i].intensity = livoxIn.points[i].reflector;
//                         pc2->points[i].line = livoxIn.points[i].line;
//                         pc2->points[i].offset_time = livoxIn.points[i].offset_time;
//                         pc2->points[i].tag = livoxIn.points[i].tag;
////                         LOG(INFO) << livoxIn.points[i].offset_time;
//                     }
//
//                     pcl::toROSMsg(*pc2, pointOut);
//                     pointOut.header.frame_id = "livox";
//                     string topic = "livox/lidar_pc2";
//                     double time = double(livoxIn.time_) / 1000000.0;
//                     pointOut.header.stamp.sec = floor(time);
//                     pointOut.header.stamp.nsec = (time - floor(time)) * 1e9;
//                     ros_bag.write(topic, pointOut.header.stamp, pointOut);
//
//                 }
//             },
//             "LIVOX_POINTS");

//  bag->setMsgHandle<sros::core::LivoxPointsMsg>
//      ([&livox_customMsg, &ros_bag](const sros::core::LivoxPointsMsg &livoxIn) {
//         //                 std::cout << "现在还在转换，时间会很长，请稍等11111111！！" << std::endl;
//         tsdf_slam::CustomMsg msgOut;
//         msgOut.points.resize(livoxIn.points.size());
//         //! Remove erroneous data
////                 if (livoxIn.time_ > 0 && livoxIn.time_ < 1e16 && !livoxIn.points.empty()) {
//         //! Loop to copy the data for each point
//         for (size_t i = 0; i < livoxIn.points.size(); i++) {
//           msgOut.points[i].x = livoxIn.points[i].x;
//           msgOut.points[i].y = livoxIn.points[i].y;
//           msgOut.points[i].z = livoxIn.points[i].z;
//           msgOut.points[i].reflectivity = livoxIn.points[i].reflector;
//           msgOut.points[i].line = livoxIn.points[i].line;
//           msgOut.points[i].offset_time = livoxIn.points[i].offset_time;
//           msgOut.points[i].tag = livoxIn.points[i].tag;
////                         LOG(INFO) << "offset time: " << msgOut.points[i].offset_time << ", "
////                                   << livoxIn.points[i].offset_time;
//         }
//
//         msgOut.point_num = livoxIn.points.size();
//         msgOut.timebase = livoxIn.time_base;
//         double time = double(livoxIn.time_) / 1000000.0;
//         msgOut.header.stamp.sec = floor(time);
//         msgOut.header.stamp.nsec = (time - floor(time)) * 1e9;
//         msgOut.header.frame_id = "livox";
//         string topic = livoxIn.topic_;
//         ros_bag.write(topic, msgOut.header.stamp, msgOut);
//         // LOG(INFO) << livoxIn.topic_ << ": " << livoxIn.time_ << ", " << msgOut.header.stamp.sec << "."
//         //           << msgOut.header.stamp.nsec << ", " << livoxIn.points.size();
////         LOG(INFO) << "/livox/lidar: " << livoxIn.time_ << ", " << livoxIn.point_num << ", " << livoxIn.points.size();
////                 }
//       },
//       "LIVOX_POINTS");
//
  bag->setMsgHandle<sros::core::ImuMsg>
      ([&imu, &ros_bag](const sros::core::ImuMsg &imuIn) {
//                 std::cout << "现在还在转换，时间会很长，请稍等2222222！！" << std::endl;
         sros::core::ImuMsg _data = imuIn;
         sensor_msgs::Imu imuOut;
         imuOut.header.frame_id = "livox/imu";
         imuOut.header.stamp.fromNSec(imuIn.time_ * 1000);
         imuOut.orientation.w = imuIn.orientation.w();
         imuOut.orientation.x = imuIn.orientation.x();
         imuOut.orientation.y = imuIn.orientation.y();
         imuOut.orientation.z = imuIn.orientation.z();

         imuOut.angular_velocity.x = imuIn.angular_velocity.x();
         imuOut.angular_velocity.y = imuIn.angular_velocity.y();
         imuOut.angular_velocity.z = imuIn.angular_velocity.z();

         imuOut.linear_acceleration.x = imuIn.linear_acceleration.x();
         imuOut.linear_acceleration.y = imuIn.linear_acceleration.y();
         imuOut.linear_acceleration.z = imuIn.linear_acceleration.z();

         for (int i = 0; i < 9; ++i) {
           imuOut.orientation_covariance[i] = imuIn.orientation_covariance[i];
           imuOut.angular_velocity_covariance[i] = imuIn.angular_velocity_covariance[i];
           imuOut.linear_acceleration_covariance[i] = imuIn.linear_acceleration_covariance[i];
         }
//         LOG(INFO) << "LIVOX_IMU: " << imuIn.time_;
         string topic = "LIVOX_IMU";
         ros_bag.write(topic, imuOut.header.stamp, imuOut);
       },
       "LIVOX_IMU");


/*!
 * \brief Transform pose msg in sros bag to ros topic msg
 * note: PoseStamedMsg is pose message obtained by the fusion of IMU and odometer
 */
  bag->setMsgHandle<sros::core::PoseStampedMsg>
      ([&poseStampedMsg, &ros_bag](const sros::core::PoseStampedMsg &posIn) {
//                 std::cout << "现在还在转换，时间会很长，请稍等333333！！" << std::endl;
         geometry_msgs::PoseStamped posOut;
//         if (posIn.time_ > 0 && posIn.time_ < 1e16) {
         posOut.header.frame_id = "odom";
         double time = double(posIn.time_) / 1000000.0;
         posOut.header.stamp.sec = floor(time);
         posOut.header.stamp.nsec = (time - floor(time)) * 1e9;
//         LOG(INFO) << posIn.topic_ << ": " << posIn.time_;
         posOut.header.seq = posIn.seq;
         tf::Quaternion q;
         q.setRPY(posIn.pose.roll(), posIn.pose.pitch(), posIn.pose.yaw());
         posOut.pose.orientation.w = q.w();
         posOut.pose.orientation.x = q.x();
         posOut.pose.orientation.y = q.y();
         posOut.pose.orientation.z = q.z();
         posOut.pose.position.x = posIn.pose.location().x();
         posOut.pose.position.y = posIn.pose.location().y();
         posOut.pose.position.z = posIn.pose.location().z();
         string topic = "OdoPoseStamped";
         ros_bag.write(topic, posOut.header.stamp, posOut);
//                     LOG(INFO) << posIn.topic_ << ", " << posIn.time_ << ", " << posOut.header.stamp.toSec();
//         }
       },
       "OdoPoseStamped");

//    bag->setMsgHandle<sros::core::PoseStampedMsg>
//            ([&poseStampedMsg, &ros_bag](const sros::core::PoseStampedMsg &matchPosIn) {
////                 std::cout << "现在还在转换，时间会很长，请稍等4444444！！" << std::endl;
//                 geometry_msgs::PoseStamped posOut;
//                 // if (matchPosIn.time_ > 0 && matchPosIn.time_ < 1e16) {
//                 posOut.header.frame_id = "map";
//                 double time = double(matchPosIn.time_) / 1000000.0;
//                 posOut.header.stamp.sec = floor(time);
//                 posOut.header.stamp.nsec = (time - floor(time)) * 1e9;
////                     LOG(INFO) << matchPosIn.topic_ << ": " << matchPosIn.time_ << ", " << posOut.header.stamp.sec << "."
////                               << posOut.header.stamp.nsec;
//                 posOut.header.seq = matchPosIn.seq;
//                 tf::Quaternion q;
//                 q.setRPY(matchPosIn.pose.roll(), matchPosIn.pose.pitch(), matchPosIn.pose.yaw());
//                 posOut.pose.orientation.w = q.w();
//                 posOut.pose.orientation.x = q.x();
//                 posOut.pose.orientation.y = q.y();
//                 posOut.pose.orientation.z = q.z();
//                 posOut.pose.position.x = matchPosIn.pose.location().x();
//                 posOut.pose.position.y = matchPosIn.pose.location().y();
//                 posOut.pose.position.z = matchPosIn.pose.location().z();
//                 string topic = "TOPIC_MATCHPOSE";
//                 ros_bag.write(topic, posOut.header.stamp, posOut);
//                 LOG(INFO) << matchPosIn.topic_ << ", " << matchPosIn.time_ << ", " << posOut.header.stamp.toSec();
//                 // }
//             },
//             "TOPIC_MATCHPOSE");

//
    bag->setMsgHandle<sros::core::LaserScanMsg>
            ([&laser, &ros_bag](const sros::core::LaserScanMsg &laserIn) {
                 std::cout << "现在还在转换，时间会很长，请稍等44444444！！" << std::endl;
                 sensor_msgs::LaserScan laserOut;

                 if (laserIn.time_ > 0 && laserIn.time_ < 1e16) {
                     double time = double(laserIn.time_) / 1000000.0;
                     laserOut.header.stamp.sec = floor(time);
                     laserOut.header.stamp.nsec = (time - floor(time)) * 1e9;

                     laserOut.header.frame_id = "laser";
                     laserOut.angle_min = laserIn.angle_min;
                     laserOut.angle_max = laserIn.angle_min;
                     laserOut.angle_increment = laserIn.angle_increment;
                     laserOut.range_min = laserIn.range_min;
                     laserOut.range_max = laserIn.range_max;
                     laserOut.scan_time = laserIn.scan_time;

                     laserOut.ranges.resize(laserIn.ranges.size());
                     laserOut.intensities.resize(laserIn.intensities.size());
                     for (size_t i = 0; i < laserIn.ranges.size(); ++i) {
                         laserOut.ranges[i] = laserIn.ranges[i];
                         laserOut.intensities[i] = laserIn.intensities[i];
                     }

                     LOG(INFO) << laserIn.topic_ << ": " << laserIn.time_ << ", " << laserOut.header.stamp.toSec();
                     ros_bag.write("UST_LEFT", laserOut.header.stamp, laserOut);

                 }
             },
             "UST_LEFT");


  bag->playBack("/home/zxj/data/sbag/2023-03-21-12-49-12.sbag", true, false);

  std::cout << "转换完成" << std::endl;

  return 0;
}

