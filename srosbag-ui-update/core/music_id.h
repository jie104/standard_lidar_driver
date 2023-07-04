
/**
 * @file music_id.h
 *
 * @author pengjiali
 * @date 20-06-16.
 *
 * @describe 所有音乐ID
 *
 * @note: 本文件是自动生成的，不要试图手动修改此文件
 *
 * @copyright Copyright (c) 2020 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef MUSIC_ID_H_
#define MUSIC_ID_H_

namespace hmi {

enum MusicID {
    MUSIC_ID_MUSIC_NONE = 0,   /*     0*/
    MUSIC_ID_MUSIC_LOW_BATTERY = 1,   /* 电量过低，请立即充电！    7*/
    MUSIC_ID_MUSIC_VERY_LOW_BATTERY = 2,   /* 电量极低，即将强制关机！    10*/
    MUSIC_ID_MUSIC_SELF_PROTECTION = 3,   /* 已强制关机，请立即充电    51*/
    MUSIC_ID_MUSIC_LOCATION_ERROR = 4,   /* 定位出错，请重新定位！    1*/
    MUSIC_ID_MUSIC_LOCATION_SUCCEED = 5,   /* 定位成功！    11*/
    MUSIC_ID_MUSIC_SWITCHING_MAP = 6,   /* 正在切换地图    6*/
    MUSIC_ID_MUSIC_SWITCH_MAP_SUCCEED = 7,   /* 地图切换完成    12*/
    MUSIC_ID_MUSIC_EMERGENCY_STOP_TRIGGER = 8,   /* 急停已触发！    9*/
    MUSIC_ID_MUSIC_EMERGENCY_STOP_RECOVERED = 9,   /* 已解除急停！    13*/
    MUSIC_ID_MUSIC_FMS_RECONNECTING = 10,   /* 调度系统连接中断，重新连接中！    14*/
    MUSIC_ID_MUSIC_FMS_RECONNECTED = 11,   /* 调度系统已连接    17*/
    MUSIC_ID_MUSIC_RECONNECTING = 12,   /* 无线网络连接异常，重新连接中！    18*/
    MUSIC_ID_MUSIC_RECONNECTED = 13,   /* 无线网络已连接    19*/
    MUSIC_ID_MUSIC_BREAK_SWITCH_ON = 14,   /* 已解除电机抱闸    20*/
    MUSIC_ID_MUSIC_FRONT_DOOR_NOT_CLOSE = 15,   /* 前门未关闭，请关闭前门    46*/
    MUSIC_ID_MUSIC_BACK_DOOR_NOT_CLOSE = 16,   /* 后门未关闭，请关闭后门    47*/
    MUSIC_ID_MUSIC_LEFT_DOOR_NOT_CLOSE = 17,   /* 左侧门未关闭，请关闭左侧门    48*/
    MUSIC_ID_MUSIC_RIGHT_DOOR_NOT_CLOSE = 18,   /* 右侧门未关闭，请关闭右侧门    49*/
    MUSIC_ID_MUSIC_SCREEN_NOT_PUT_BACK_IN_PLACE = 19,   /* 显示屏未放置到位，请检查    50*/
    MUSIC_ID_MUSIC_LIDAR_ERROR = 20,   /* 导航雷达故障，请检查硬件状态    52*/
    MUSIC_ID_MUSIC_OBSTACLE_AVOID_LIDAR_ERROR = 21,   /* 避障雷达故障，请检查硬件状态    53*/
    MUSIC_ID_MUSIC_MOTOR_ERROR = 22,   /* 电机故障，请检查硬件状态    54*/
    MUSIC_ID_MUSIC_SRC_DISCONNECT = 23,   /* 运动控制程序掉线    55*/
    MUSIC_ID_MUSIC_SRC_ERROR = 24,   /* 运动控制程序报错，请查阅故障信息    56*/
    MUSIC_ID_MUSIC_TURN_LEFT = 25,   /* 左转弯，请注意！    21*/
    MUSIC_ID_MUSIC_TURN_RIGHT = 26,   /* 右转弯，请注意！    22*/
    MUSIC_ID_MUSIC_BACK_UP = 27,   /* 倒车，请注意！    3*/
    MUSIC_ID_MUSIC_ARRIVED = 28,   /* 已到站    8*/
    MUSIC_ID_MUSIC_OFF_COURSE = 29,   /* 偏离轨迹过大，已停止    23*/
    MUSIC_ID_MUSIC_PATH_REPLANNING = 30,   /* 正在重新规划路径    24*/
    MUSIC_ID_MUSIC_PATH_PLANING_FAILED = 31,   /* 规划路径失败    25*/
    MUSIC_ID_MUSIC_OBSTACLE_AHEAD = 32,   /* 前方有障碍    2*/
    MUSIC_ID_MUSIC_OBSTACLE_BEHIND = 33,   /* 后方有障碍    4*/
    MUSIC_ID_MUSIC_IN_TRAFFIC_CONTROL = 34,   /* 等待交管指令    26*/
    MUSIC_ID_MUSIC_NORMAL_RUNNING_A = 35,   /* 音乐“beep”    16*/
    MUSIC_ID_MUSIC_NORMAL_RUNNING_B = 36,   /* 音乐“beep”    27*/
    MUSIC_ID_MUSIC_NORMAL_RUNNING_C = 37,   /* 音乐“beep”    28*/
    MUSIC_ID_MUSIC_NORMAL_RUNNING_D = 38,   /* 音乐“beep”    29*/
    MUSIC_ID_MUSIC_NORMAL_RUNNING_E = 39,   /* 音乐“beep”    57*/
    MUSIC_ID_MUSIC_NORMAL_RUNNING_0 = 40,   /* 音乐“beep”    200*/
    MUSIC_ID_MUSIC_NORMAL_RUNNING_1 = 41,   /* 音乐“beep”    201*/
    MUSIC_ID_MUSIC_NORMAL_RUNNING_2 = 42,   /* 音乐“beep”    202*/
    MUSIC_ID_MUSIC_NORMAL_RUNNING_3 = 43,   /* 音乐“beep”    203*/
    MUSIC_ID_MUSIC_NORMAL_RUNNING_4 = 44,   /* 音乐“beep”    204*/
    MUSIC_ID_MUSIC_NORMAL_RUNNING_5 = 45,   /* 音乐“beep”    205*/
    MUSIC_ID_MUSIC_NORMAL_RUNNING_6 = 46,   /* 音乐“beep”    206*/
    MUSIC_ID_MUSIC_NORMAL_RUNNING_7 = 47,   /* 音乐“beep”    207*/
    MUSIC_ID_MUSIC_NORMAL_RUNNING_8 = 48,   /* 音乐“beep”    208*/
    MUSIC_ID_MUSIC_NORMAL_RUNNING_9 = 49,   /* 音乐“beep”    209*/
    MUSIC_ID_MUSIC_OBSTACLE_ON_THE_LEFT = 50,   /* 左边有障碍    58*/
    MUSIC_ID_MUSIC_OBSTACLE_ON_THE_RIGHT = 51,   /* 右边有障碍    59*/
    MUSIC_ID_MUSIC_OBSTACLE_ON_THE_LEFT_FRONT = 52,   /* 左前方有障碍    60*/
    MUSIC_ID_MUSIC_OBSTACLE_ON_THE_LEFT_REAR = 53,   /* 左后方有障碍    61*/
    MUSIC_ID_MUSIC_OBSTACLE_ON_THE_RIGHT_FRONT = 54,   /* 右前方有障碍    62*/
    MUSIC_ID_MUSIC_OBSTACLE_ON_THE_RIGHT_REAR = 55,   /* 右后方有障碍    63*/
    MUSIC_ID_MUSIC_ENTER_NO_ENTRY_AREA = 56,   /* 进入禁止进入区域了    64*/
    MUSIC_ID_MUSIC_OBSTACLE_AVOID_FUNCTION_IS_OFF = 57,   /* 避障功能临时关闭，请注意    65*/
    MUSIC_ID_MUSIC_WAITING_ELEVATOR = 58,   /* 正在等待电梯到达    5*/
    MUSIC_ID_MUSIC_NO_GROUND_DM_CODE_DETECTED = 59,   /* 未检测到地面二维码    30*/
    MUSIC_ID_MUSIC_RACK_DM_CODE_DETECTED = 60,   /* 未检测到货架二维码    15*/
    MUSIC_ID_MUSIC_ACTION_NORMAL_RUNNING = 61,   /* 正在执行动作或背景音乐    31*/
    MUSIC_ID_MUSIC_RACK_JACK_FAILED = 62,   /* 顶起货架失败    32*/
    MUSIC_ID_MUSIC_RACK_PUT_DOWN_FAILED = 63,   /* 放下货架失败    33*/
    MUSIC_ID_MUSIC_RACK_ROTATION = 64,   /* 带货架旋转，请远离    66*/
    MUSIC_ID_MUSIC_COMMUNICATION_RESPONSE_TIMEOUT = 66,   /* 通信应答超时    35*/
    MUSIC_ID_MUSIC_DOCK_ERROR = 68,   /* 对方机台设备故障    37*/
    MUSIC_ID_MUSIC_DOCK_ALREADY_HAS_GOODS = 69,   /* 当前工位已有物料，无法送料    38*/
    MUSIC_ID_MUSIC_DOCK_NO_GOODS = 70,   /* 当前工位没有物料，无法取料    39*/
    MUSIC_ID_MUSIC_PICKING_IS_STUCK = 71,   /* 机器人取料卡料    40*/
    MUSIC_ID_MUSIC_FEED_IS_STUCK = 72,   /* 机器人送料卡料    41*/
    MUSIC_ID_MUSIC_GOODS_TRANSMISSION_TIMEOUT = 73,   /* 货物传输超时    42*/
    MUSIC_ID_MUSIC_GOODS_TRANSMISSION_SUCCEED = 74,   /* 货物传输完成    43*/
    MUSIC_ID_MUSIC_AGV_ARRIVED_PLACE_FEED_TO_DOCK = 75,   /* 已到站，请送料    44*/
    MUSIC_ID_MUSIC_AGV_ARRIVED_PLACE_PICKING_FROM_DOCK = 76,   /* 已到站，请取料    45*/
    MUSIC_ID_MUSIC_GOODS_EXCEED_THE_LIMIT = 77,   /* 货物超出限位，请检查    67*/
    MUSIC_ID_MUSIC_GOODS_PLACEMENT_POINT_IS_OCCUPIED = 78,   /* 货物放置点被占用    68*/
    MUSIC_ID_MUSIC_WAITING_FOR_LAUNCH = 79,   /* 等待放行中    69*/
    MUSIC_ID_MUSIC_MISSION_PAUSED = 80,   /* 任务被暂停    70*/
    MUSIC_ID_MUSIC_EAC_TIMEOUT = 81,   /* 扩展动作控制器超时    71*/
    MUSIC_ID_MUSIC_EAC_ERROR = 82,   /* 扩展动作控制器异常    72*/
    MUSIC_ID_MUSIC_ROBOTIC_ARM_TIMEOUT = 83,   /* 机械臂超时    73*/
    MUSIC_ID_MUSIC_ROBOTIC_ARM_ERROR = 84,   /* 机械臂异常    74*/
    MUSIC_ID_MUSIC_FIND_AGV = 85,   /* 寻找机器人    75*/
    MUSIC_ID_MUSIC_DISINGECTING = 100,   /* 正在消毒中，请注意个人防护    100*/

};

}  // namespace hmi

#endif  // MUSIC_ID_H_

