
/**
 * @file fault_code.h
 *
 * @author pengjiali
 * @date 20-06-15.
 *
 * @describe SROS 故障码头文件
 *
 * @note: 本文件是自动生成的，不要试图手动修改此文件
 *
 * @copyright Copyright (c) 2020 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef FAULT_CODE_H_
#define FAULT_CODE_H_

namespace sros {
namespace core {

enum FaultCode {
    FAULT_CODE_NONE = 0, // 没错误
    FAULT_CODE_CPU_LOAD_IS_TOO_HIGH = 1310,   /* cup占用率连续三分钟超过90%，可能是软件异常导致。    断电重启车辆。若重启车辆后还继续出现，请联系厂家。    MUSIC_NONE*/
    FAULT_CODE_CPU_PEAK_LOAD_IS_TOO_HIGH = 1211,   /* cpu峰值过高，可能会影响软件运行。    断电重启车辆。若重启车辆后还继续出现，请联系厂家。    MUSIC_NONE*/
    FAULT_CODE_FREE_MEMORY_IS_VERY_LOW = 1312,   /* 剩余可用内存过低，可能是软件异常导致。    断电重启车辆。若重启车辆后还继续出现，请联系厂家。    MUSIC_NONE*/
    FAULT_CODE_FREE_DISK_IS_VERY_LOW = 1313,   /* 剩余可用硬盘过低，可能是日志太多导致的。    断电重启车辆。若重启车辆后还继续出现，请联系厂家。    MUSIC_NONE*/
    FAULT_CODE_CPU_TEMPERATURE_IS_TOO_HIGH = 1214,   /* cpu温度过高    断电重启车辆。若重启车辆后还继续出现，请联系厂家。    MUSIC_NONE*/
    FAULT_CODE_BOARD_TEMPERATURE_IS_TOO_HIGH = 1215,   /* 主板温度过高    断电重启车辆。若重启车辆后还继续出现，请联系厂家。    MUSIC_NONE*/
    FAULT_CODE_BATTERY_IS_TOO_LOW = 11210,   /* 电池电量过低，即将关机    请给车辆充电。    MUSIC_LOW_BATTERY*/
    FAULT_CODE_BATTERY_IS_VERY_LOW = 11211,   /* 电池电量小于10%，即将关机    请给车辆充电。    MUSIC_VERY_LOW_BATTERY*/
    FAULT_CODE_BREAK_SWITCH_ON = 11312,   /* 抱闸开关被打开    请将电源开关打到第三档。    MUSIC_BREAK_SWITCH_ON*/
    FAULT_CODE_SCREEN_IS_TURNED_UP = 11313,   /* 触摸屏被翻起来了    请将触摸屏放下。    MUSIC_SCREEN_NOT_PUT_BACK_IN_PLACE*/
    FAULT_CODE_POWER_SAVE_MODE = 11314,   /* 进入低功耗模式    请退出低功耗模式    MUSIC_NONE*/
    FAULT_CODE_ODO_TIMEOUT = 11415,   /* odo超时    请联系厂家    MUSIC_NONE*/
    FAULT_CODE_UPSVC100_SCAN_FAIL = 11416,   /* 上视svc100扫码失败    请联系厂家    MUSIC_NONE*/
    FAULT_CODE_EMERGENCY_TRIGGER_BUTTON = 11321,   /* 急停按钮触发    请将急停按钮解除，然后继续    MUSIC_EMERGENCY_STOP_TRIGGER*/
    FAULT_CODE_EMERGENCY_TRIGGER_EDGE = 11322,   /* 急停触边触发    请将排除触边的东西后，然后继续    MUSIC_EMERGENCY_STOP_TRIGGER*/
    FAULT_CODE_EMERGENCY_TRIGGER_SOFTWARE = 11323,   /* 软件急停触发    请解除急停后继续    MUSIC_EMERGENCY_STOP_TRIGGER*/
    FAULT_CODE_EMERGENCY_TRIGGER_BATTERY_DOOR_OPEN = 11324,   /* 电池舱门打开触发急停    请关闭电池舱门后，解除急停后    MUSIC_EMERGENCY_STOP_TRIGGER*/
    FAULT_CODE_EMERGENCY_TRIGGER_UNKNOWN = 11325,   /* 急停触发    请解除急停后    MUSIC_EMERGENCY_STOP_TRIGGER*/
    FAULT_CODE_EMERGENCY_TRIGGER_NXP_ESTOP = 11326,   /* 硬件NXP触发    请解除急停后继续    MUSIC_EMERGENCY_STOP_TRIGGER*/
    FAULT_CODE_EMERGENCY_TRIGGER_ST_ESTOP = 11327,   /* 硬件STM32触发    请解除急停后继续    MUSIC_EMERGENCY_STOP_TRIGGER*/
    FAULT_CODE_EMERGENCY_TRIGGER_TK1_ESTOP = 11328,   /* 硬件TK1触发    请解除急停后继续    MUSIC_EMERGENCY_STOP_TRIGGER*/
    FAULT_CODE_EMERGENCY_TRIGGER_ESTOP_OUT = 11329,   /* 硬件急停锁存电路触发    请解除急停后继续    MUSIC_EMERGENCY_STOP_TRIGGER*/
    FAULT_CODE_LOCATION_ERROR = 11330,   /* 定位出错    请重新定位    MUSIC_LOCATION_ERROR*/
    FAULT_CODE_ENTER_NO_ENTRY_AREA = 11331,   /* 车辆进入了禁止进入区域    请确认当前车辆位置是否正常，若车辆位置不正常请排查车辆为什么会进入禁止进入区域，并将车辆推出禁止进入区域；
若车辆位置正常，请重新绘制合理的禁止进入区域；    MUSIC_ENTER_NO_ENTRY_AREA*/
    FAULT_CODE_FMS_DISCONNECT = 11332,   /* 当前开启调度模式，但fms用户和车辆断开了链接    请保持fms和车辆正常链接；或是退出调度模式，让车辆单机运行；    MUSIC_FMS_RECONNECTING*/
    FAULT_CODE_SCREEN_TURNED_UP = 11333,   /* 触摸屏被翻转起来了    请将触摸屏按下去    MUSIC_SCREEN_NOT_PUT_BACK_IN_PLACE*/
    FAULT_CODE_EMERGENCY_TRIGGER_SRC_INTERNAL_FAULT = 11334,   /* SRC内部自定义故障    请解除急停后继续    MUSIC_EMERGENCY_STOP_TRIGGER*/
    FAULT_CODE_EMERGENCY_TRIGGER_SRC_EXTERNAL_FAULT = 11335,   /* SRC外部自定义故障    请解除急停后继续    MUSIC_EMERGENCY_STOP_TRIGGER*/
    FAULT_CODE_MODBUS_RTU_DEVICE_CAN_NOT_OPEN = 11250,   /* modbus-RTU硬件无法打开    请确认当前是否需要modbus-RTU，若不需要将参数modbus.enable_modbus_rtu设置为False；
若需要请检测设备是否插上、参数是否配置正确    MUSIC_NONE*/
    FAULT_CODE_FORK_LIFT_FAULT = 11451,   /* 空货叉升降故障    请检测空货叉升降叉功能是否正常，待人工处理好后，重试当前动作    MUSIC_NONE*/
    FAULT_CODE_GOODS_NOT_DETECTED = 11452,   /* 货架上未检测到货物    请确认前方货架上是否有货，待人工处理好后，重试当前动作    MUSIC_NONE*/
    FAULT_CODE_GOODS_POS_YAW_DEVIATE = 11453,   /* 3D视觉相机检测到货物摆放角度太偏    请检查货物摆放角度是否过偏并调整角度摆正货物，待人工处理好后，重试当前动作    MUSIC_NONE*/
    FAULT_CODE_GOODS_POS_X_DEVIATE_FORWARD = 11454,   /* 3D视觉相机检测到货物X方向太靠前    请检查实际货物摆放X方向是否太靠前并摆正，待人工处理好后，重试当前动作    MUSIC_NONE*/
    FAULT_CODE_GOODS_POS_X_DEVIATE_BACK = 11455,   /* 3D视觉相机检测到货物X方向太靠后    请检查实际货物摆放X方向是否太靠后并摆正，待人工处理好后，重试当前动作    MUSIC_NONE*/
    FAULT_CODE_GOODS_POS_Y_DEVIATE = 11456,   /* 3D视觉相机检测到货物摆放Y方向太偏    请检查货物Y方向摆放是否过偏并摆正货物，待人工处理好后，重试当前动作    MUSIC_NONE*/
    FAULT_CODE_LOAD_NO_DOCKING = 11457,   /* 货叉取货路径对接失败    请排查原因并手动将货叉对接好货物，待人工处理好后，跳过当前动作    MUSIC_NONE*/
    FAULT_CODE_FORK_FAIL_PICKUP_GOODS = 11458,   /* 载货失败    请检查叉臂载货上升功能是否正常并手动装载货物，待人工处理好后，跳过当前动作    MUSIC_NONE*/
    FAULT_CODE_BACKFORK_MOVE_NOT_REACH = 11459,   /* 取货退出路径导航失败    请排查失败原因并手动导航到目标站点，待人工处理好后，重试当前动作    MUSIC_NONE*/
    FAULT_CODE_GOODS_DETECTED = 11460,   /* 卸货位上有货物    请检查当前卸货位是否有货物，待人工处理好后，重试当前动作    MUSIC_NONE*/
    FAULT_CODE_UNLOAD_MOVE_NOT_REACH = 11461,   /* 卸货路径导航失败    请排查失败原因并手动导航到目标站点，待人工处理好后，重试当前动作    MUSIC_NONE*/
    FAULT_CODE_LOAD_PALLET_BOUNCE = 11462,   /* 载货时到位开关未触发    请排查失败原因并确保货物在货叉正中心位置，紧贴叉臂根部，待人工处理好后，重试当前动作    MUSIC_NONE*/
    FAULT_CODE_FORK_FAIL_PICKDOWN_GOODS = 11463,   /* 卸货叉臂下降失败    请检查卸货叉臂下降功能是否正常并手动卸载货物，待人工处理好后，跳过当前动作    MUSIC_NONE*/

    FAULT_CODE_EAC_UNKNOWN_FAULT = 931400,
    FAULT_CODE_EAC_CONNECT_FAILED = 931401,
    FAULT_CODE_EAC_INITIAL_FAILED = 931402,
    FAULT_CODE_EAC_TIMEOUT = 931403,
    FAULT_CODE_EAC_OTHER_FAULT = 931499,
};

enum FaultResponseBehavior {
    FAULT_RESPONSE_BEHAVIOR_NONE = 0,   /* 没有响应*/
    FAULT_RESPONSE_BEHAVIOR_ACTION_DISABLE_PAUSE_MANUAL_CONTINUE = 11,   /* 当故障出现时不能启动动作任务，暂停正在执行的动作任务，当故障恢复后需要人为确认后才能继续*/
    FAULT_RESPONSE_BEHAVIOR_ACTION_DISABLE_PAUSE_AUTO_CONTINUE = 12,   /* 当故障出现时不能启动动作任务，暂停正在执行的动作任务，当故障恢复后自动继续*/
    FAULT_RESPONSE_BEHAVIOR_MOVEMENT_DISABLE_PAUSE_MANUAL_CONTINUE = 81,   /* 当故障出现时不能启动移动任务，暂停正在执行的移动任务，当故障恢复后需要人为确认后才能继续*/
    FAULT_RESPONSE_BEHAVIOR_MOVEMENT_DISABLE_PAUSE_AUTO_CONTINUE = 82,   /* 当故障出现时不能启动移动任务，暂停正在执行的移动任务，当故障恢复后自动继续*/
    FAULT_RESPONSE_BEHAVIOR_ALL_DISABLE_PAUSE_MANUAL_CONTINUE = 101,   /* 当故障出现时移动和动作任务都不能启动，暂停正在执行想任务，当故障恢复后需要人为确认后才能继续*/
    FAULT_RESPONSE_BEHAVIOR_ALL_DISABLE_PAUSE_AUTO_CONTINUE = 102,   /* 当故障出现时移动和动作任务都不能启动，暂停正在执行想任务，当故障恢复后自动继续*/
    FAULT_RESPONSE_BEHAVIOR_EMERGENCY = 201,   /* 当出现故障时，直接触发急停*/

};    

}  // namespace core
}  // namespace sros

#endif  // FAULT_CODE_H_

