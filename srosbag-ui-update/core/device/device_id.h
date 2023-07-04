
/**
 * @file device_id.h
 *
 * @author pengjiali
 * @date 20-06-16.
 *
 * @describe 所有设备ID
 *
 * @note: 本文件是自动生成的，不要试图手动修改此文件
 *
 * @copyright Copyright (c) 2020 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef DEVICE_ID_H_
#define DEVICE_ID_H_

namespace sros {
namespace device {

// 标识唯一的设备ID，由三位十位数组成,这个用于统一故障码，编码需要在sros-resources参考中完成
enum DeviceID {
    DEVICE_ID_UNDEFINED = 0,  // 未定义
    DEVICE_ID_VC300 = 1,   /* VC300    /   */
    DEVICE_ID_SROS = 11,   /* SROS    /   */
    DEVICE_ID_LIDAR = 111,   /* R2000    SROS   ETH_1*/
    DEVICE_ID_IMU = 121,   /* IMU    SRC   RS232_2*/
    DEVICE_ID_SVC100_UP = 131,   /* SVC100    SROS   USB*/
    DEVICE_ID_SVC100_DOWN = 132,   /* SVC100    SROS   USB*/
    DEVICE_ID_SVC200_BACKWARD = 133,   /* SVC100    SROS   USB*/
    DEVICE_ID_SVC200_LEFT = 134,   /* SVC200    SROS   USB*/
    DEVICE_ID_SVC200_RIGHT = 135,   /* SVC200    SROS   USB*/
    DEVICE_ID_SVC200_BACK = 136,   /* SVC200    SROS   USB*/
    DEVICE_ID_SVC200_DOWN = 137,   /* SVC200    SROS   USB*/
    DEVICE_ID_SVC200_UP = 138,   /* SVC200    SROS   USB*/
    DEVICE_ID_MC_MOTOR_1 = 211,   /* MOTOR    SRC   CAN_2*/
    DEVICE_ID_MC_MOTOR_2 = 212,   /* MOTOR    SRC   CAN_2*/
    DEVICE_ID_MC_MOTOR_3 = 213,   /* MOTOR    SRC   CAN_2*/
    DEVICE_ID_MC_MOTOR_4 = 214,   /* MOTOR    SRC   CAN_2*/
    DEVICE_ID_PGV_UP = 221,   /* PGV    SRC   RS232_2*/
    DEVICE_ID_PGV_DOWN = 222,   /* PGV    SRC   RS232_2*/
    DEVICE_ID_SRC = 231,   /* SRC    SROS   RS232_2*/
    DEVICE_ID_PMU = 411,   /* PMU    SROS   RS232_3*/
    DEVICE_ID_BMS = 421,   /* BMS    SROS   RS232_3*/
    DEVICE_ID_LC100 = 511,   /* LC100    SROS   CAN_1*/
    DEVICE_ID_LC100_2 = 512,   /* LC100    SROS   CAN_1*/
    DEVICE_ID_SPEAKER = 521,   /* SPEAKER    SROS   CAN_1*/
    DEVICE_ID_SCREEN = 531,   /* SCREEN    SROS   RS232_1*/
    DEVICE_ID_R2100 = 611,   /* R2100    SROS   ETH_1*/
    DEVICE_ID_CAMERA_D435 = 621,   /* CAMERA_D435    SROS   USB*/
    DEVICE_ID_CAMERA_D435_2 = 622,   /* CAMERA_D435    SROS   USB*/
    DEVICE_ID_CAMERA_O3D303 = 623,   /* CAMERA_O3D303    SROS   ETH_1*/
    DEVICE_ID_LIVOX_MIDXX = 624,   /* LIVOX_MIDXX    SROS   ETH_1*/
    DEVICE_ID_SH100_TOF_1 = 631,   /* SH100_TOF    SROS   CAN_1*/
    DEVICE_ID_SH100_TOF_2 = 632,   /* SH100_TOF    SROS   CAN_1*/
    DEVICE_ID_EU100_TIM312_1 = 641,   /* EU100_TIM312    SROS   CAN_1*/
    DEVICE_ID_EU100_TIM312_2 = 642,   /* EU100_TIM312    SROS   CAN_1*/
    DEVICE_ID_EU100_TIM312_3 = 643,   /* EU100_TIM312    SROS   CAN_1*/
    DEVICE_ID_UST05_1 = 644,   /* UST05    SROS   ETH_1*/
    DEVICE_ID_UST05_2 = 645,   /* UST05    SROS   ETH_1*/
    DEVICE_ID_UST05_3 = 646,   /* UST05    SROS   ETH_1*/
    DEVICE_ID_VSC = 711,   /* VSC    SROS   RS232_3*/
    DEVICE_ID_SPU = 712,   /* SPU    SROS   RS232_3*/
    DEVICE_ID_AC_MOTOR_1 = 911,   /* MOTOR    SRC   CAN_2*/
    DEVICE_ID_AC_MOTOR_2 = 912,   /* MOTOR    SRC   CAN_2*/
    DEVICE_ID_AC_MOTOR_3 = 913,   /* MOTOR    SRC   CAN_2*/
    DEVICE_ID_AC_MOTOR_4 = 914,   /* MOTOR    SRC   CAN_2*/
    DEVICE_ID_EU100_1 = 921,   /* EU100    SRC   CAN_2*/
    DEVICE_ID_EU100_2 = 922,   /* EU100    SRC   CAN_2*/
    DEVICE_ID_EU100_3 = 923,   /* EU100    SRC   CAN_2*/
    DEVICE_ID_EU100_4 = 924,   /* EU100    SRC   CAN_2*/
    DEVICE_ID_EAC = 931,   /* EAC    SROS   ETH_1*/

};

}  // namespace device
}  // namespace sros

#endif  // DEVICE_ID_H_

