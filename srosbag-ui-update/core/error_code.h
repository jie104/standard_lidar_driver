
/**
 * @file error_code.h
 *
 * @author pengjiali
 * @date 20-04-08.
 *
 * @describe sros 错误码头文件
 *
 * @note: 本文件是自动生成的，不要试图手动修改此文件
 *
 * @copyright Copyright (c) 2020 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef ERROR_CODE_H_
#define ERROR_CODE_H_

namespace sros {
namespace core {

enum ErrorCode {
    ERROR_CODE_NONE = 0,   /* 没有任何错误     无*/
    ERROR_CODE_USER_OR_PASSWORD_INVALID = 101,   /* 连接失败，账号或密码错误    请用正确的账号和密码登录*/
    ERROR_CODE_USER_FMS_ALREADY_EXISTS = 102,   /* FMS用户已经登录了，不允许其他的FMS用户登录    请检查是否有其他FMS用户登录了，或者是FMS掉线后重连没有带上一次的session_id*/
    ERROR_CODE_UNDEFINED = 200,   /* 未定义的错误码    联系厂家*/
    ERROR_CODE_UNREACHABLE = 201,   /* unreachable    联系厂家*/
    ERROR_CODE_CONTROL_MUTEX_IS_LOCKED = 300,   /* 车辆当前被他人独占，不允许对车辆进行“写”操作    等待独占者解除独占，或者用admin权限强制解除他人的独占*/
    ERROR_CODE_EXECUTE_COMMAND_IN_INITIALIZING = 301,   /* 车辆正在初始化，不能执行命令。    等车辆初始化完成后，再试*/
    ERROR_CODE_FUNCTION_ABANDONED = 302,   /* 该功能被废弃    请用最新的协议*/
    ERROR_CODE_SRC_NOT_SUPPORE = 303,   /* SRC不支持该功能    请找项目经理确认SRC需要哪个版本，升级完后再试*/
    ERROR_CODE_LOAD_MAP_FAILED = 1000,   /* 加载地图失败    请检查地图是否正确*/
    ERROR_CODE_UPGRADE_SROS_FAILED_CURRENT_VERSION_MUST_MORE_THAN_4010001 = 2001,   /* 升级版本失败    请先升级v4.10.1然后升级此版本*/
    ERROR_CODE_UPGRADE_SROS_FAILED_NEW_VERSION_MUST_MORE_THAN_4010001 = 2010,   /* 升级版本失败    升级的版本必须大于v4.10.1*/
    ERROR_CODE_LOCATION_SLAM_STATE_ERROR = 10002,   /* 启动定位时SLAM状态异常    稍后再试*/
    ERROR_CODE_LOCATION_STATION_NOT_EXIST = 10003,   /* 通过站点定位，但是该站点不存在    确保车辆在站点上，才能通过站点定位*/
    ERROR_CODE_LOCATION_NO_MAP = 10005,   /* 定位的时候地图未加载    请加载地图之后，再试*/
    ERROR_CODE_LOCATION_MAP_LOAD_ERROR = 10006,   /* 地图没有正确加载，无法定位    请检查地图是否正确后，再试*/
    ERROR_CODE_LOCATION_IN_MAP_SWITCHING = 10007,   /* 定位时正在切换地图    等地图切换后，再试*/
    ERROR_CODE_LOCATION_MOVEMENT_RUNNING = 20003,   /* 关闭定位时车辆正在执行移动任务    等车辆空闲后，再试*/
    ERROR_CODE_LOCATION_MISSION_RUNNING = 20004,   /* 关闭定位时车辆正在执行mission任务    等车辆空闲后，再试*/
    ERROR_CODE_STOP_LOCATION_IN_MAP_SWITCHING = 20007,   /* 关闭定位时正在切换地图    等地图切换后，再试*/
    ERROR_CODE_CANCEL_EMERGENCY_CAN_NOT_RECOVER = 30001,   /* 由于有急停不可解除，导致解除急停失败    恢复急停触源后（如：恢复急停开关），再试*/
    ERROR_CODE_SET_SPEAKER_VOLUME_PARAM_ERROR = 37001,   /* 设置喇叭音量时，参数出错    请重新读取协议文档，再试*/
    ERROR_CODE_CONTINUE_IN_EMERGENCY = 40001,   /* 继续时，当前已经处于急停状态    解除急停后，再试*/
    ERROR_CODE_CONTINUE_IN_NOT_LOCATION = 40002,   /* 继续时车辆处于未定位状态    定位后，再试*/
    ERROR_CODE_CONTINUE_IN_NOT_PAUSE = 40003,   /* 继续时车辆处于暂停状态    当前没处于暂停状态，无法继续*/
    ERROR_CODE_CONTINUE_IN_FAULT = 40004,   /* 继续时车辆存在一些无法继续的故障    请排查当前故障后，再试*/
    ERROR_CODE_SET_HMI_STATE_PARAM_ERROR = 41001,   /* 设置系统HMI时参数出错    请重新读取协议文档，再试*/
    ERROR_CODE_SYNC_TIME_SYSTEM_NOT_IDLE = 50001,   /* 设置系统时间的时候，系统处于繁忙状态    等车辆空闲后，再试*/
    ERROR_CODE_SYNC_TIME_MISSION_RUNNING = 50005,   /* 设置系统时间的时候，mission 正在运行    等车辆空闲后，再试*/
    ERROR_CODE_SYNC_TIME_NTP_ENABLE = 50010,   /* 开启时间同步功能（NTP）后不手动设置时间    若要手动设置时间请将NTP功能关闭（time.enable_ntp_sync）后，再试*/
    ERROR_CODE_MANUAL_CONTROL_NOT_ON = 70001,   /* 手动控制车辆时，车辆不处于手动控制状态    设置车辆为手动控制模式后，再试*/
    ERROR_CODE_MANUAL_CONTROL_IN_EMERGENCY = 70002,   /* 车辆处于急停状态下启动手动控制    解除急停后，再试*/
    ERROR_CODE_MANUAL_CONTROL_IN_BREAK_SW = 70003,   /* 车辆处于解抱闸状态下启动手动控制    请取消解抱闸（电源开关打到第三档），再试*/
    ERROR_CODE_MANUAL_CONTROL_MOVEMENT_RUNNING = 70004,   /* 有移动任务执行不能切换到手动控制    等车辆空闲后，再试*/
    ERROR_CODE_MANUAL_CONTROL_MISSION_RUNNING = 70005,   /* mission任务正在执行时不能切换手动控制    等车辆空闲后，再试*/
    ERROR_CODE_NEW_MAP_MOVEMENT_RUNNING = 80001,   /* 移动任务在执行的时候不能创建地图    等车辆空闲后，再试*/
    ERROR_CODE_NEW_MAP_MISSION_RUNNING = 80002,   /* mission任务在执行的时候不能创建地图    等车辆空闲后，再试*/
    ERROR_CODE_NEW_MAP_ALREADY_STARTED = 80003,   /* 绘制新地图时，已经开始绘制地图了    不要发送重复的命令，无需处理*/
    ERROR_CODE_MAP_RENAME_ORIGINAL_NAME_IS_EMPTY = 80081,   /* 地图重命名时，要重命名的地图名为空    请检测参数是否正确，再试*/
    ERROR_CODE_MAP_RENAME_DESTINATION_NAME_IS_EMPTY = 80082,   /* 地图重命名时，目标地图名为空    请检测参数是否正确，再试*/
    ERROR_CODE_MAP_RENAME_CURRENT_MAP_IN_USE = 80083,   /* 地图重命名时，目标地图正在使用    请切换完当前地图，再试*/
    ERROR_CODE_SET_MAP_SYSTEM_NOT_IDLE = 100001,   /* 设置使用当前地图时系统处于非空闲状态    等车辆空闲后，再试*/
    ERROR_CODE_SET_MAP_MOVEMENT_RUNNING = 100002,   /* 设置使用当前地图时有移动任务执行    等车辆空闲后，再试*/
    ERROR_CODE_SET_MAP_MISSION_RUNNING = 100003,   /* 设置使用当前地图时mission任务正在执行    等车辆空闲后，再试*/
    ERROR_CODE_SET_MAP_LOCATION_NOT_NONE = 100004,   /* 设置使用当前地图时处于定位状态    取消定位后，再试*/
    ERROR_CODE_SET_MAP_MAP_LOAD_ERROR = 100005,   /* 设置当前地图时，当前地图不存在或者是当前地图数据被破坏    请确认当前地图名是否正确，再试*/
    ERROR_CODE_SET_CONFIG_MOVEMENT_RUNNING = 110001,   /* 设置参数的时移动任务在运行    等车辆空闲后，再试*/
    ERROR_CODE_SET_CONFIG_ACTION_RUNNING = 110002,   /* 设置参数的时动作任务在运行    等车辆空闲后，再试*/
    ERROR_CODE_SET_CONFIG_MISSION_RUNNING = 110003,   /* 设置参数的时mission任务在运行    等车辆空闲后，再试*/
    ERROR_CODE_RESULT_NOT_MATCH_TASK_PARAM_STR = 110004,   /* 执行结果返回串与动作下发的匹配串不一致    请确认是否匹配失败，再试*/
    ERROR_CODE_SET_SPEED_NOT_IN_MANUAL = 160001,   /* 车辆不处于手动控制状态，不能手动控制车辆    设置车辆为手动控制模式后，再试*/
    ERROR_CODE_LOCK_CONTROL_MUTEX_MOVEMENT_RUNNING = 180001,   /* 获取独占模式时移动任务正在运行    等车辆空闲后，再试*/
    ERROR_CODE_LOCK_CONTROL_MUTEX_ACTION_RUNNING = 180002,   /* 获取独占模式时动作任务正在运行    等车辆空闲后，再试*/
    ERROR_CODE_LOCK_CONTROL_MUTEX_MISSION_RUNNING = 180003,   /* 获取独占模式时mission任务正在运行    等车辆空闲后，再试*/
    ERROR_CODE_LOCK_CONTROL_MUTEX_NONE_NICK_NAME = 180004,   /* 获取独占模式时没有带上锁人的绰号    带上绰号后，再试*/
    ERROR_CODE_LOCK_CONTROL_MUTEX_NONE_IP_ADDRESS = 180005,   /* 获取独占模式时没有带上锁人的ip    带上ip后，再试*/
    ERROR_CODE_UNLOCK_CONTROL_MUTEX_CURRENT_IS_UNLOCKED = 190001,   /* 释放独占模式时当前已经是非独占模式    */
    ERROR_CODE_UNLOCK_CONTROL_MUTEX_SESSION_ID_MISMATCH = 190002,   /* 释放独占模式时sessionId不匹配    只有独占着才能解除独占模式，非独占者可以用CMD_FORCE_UNLOCK_CONTROL_MUTEX命令强制解除他人的独占*/
    ERROR_CODE_FORCE_UNLOCK_CONTROL_MUTEX_PERMISSION_DENIED = 200001,   /* 强制释放独占模式时权限不够    只有admin及以上的全权限才能强制解除他人独占*/
    ERROR_CODE_FORCE_UNLOCK_CONTROL_MUTEX_FLEET_DENIED = 200002,   /* 强制释放独占模式时当前处于调度我们，无法解除    退出调度模式就能解除调度独占*/
    ERROR_CODE_MOVEMENT_IN_EMERGENCY = 320001,   /* 启动移动任务时，车辆处于急停状态    解除急停后，再试*/
    ERROR_CODE_MOVEMENT_NO_LOCATION = 320002,   /* 启动移动任务时，车辆不处于定位状态    定位后，再试*/
    ERROR_CODE_MOVEMENT_PRE_TASK_RUNNING = 320003,   /* 启动移动任务时，上一个任务在运行    等车辆空闲后，再试*/
    ERROR_CODE_MOVEMENT_INVALID_CMD_PARAM = 320004,   /* 启动移动任务时，参数错误    请检测任务参数后，再试*/
    ERROR_CODE_MOVEMENT_IN_MANUAL_CONTROL = 320005,   /* 启动移动任务时，车辆处于手动控制状态    请将车辆设置为自动控制模式后，再试*/
    ERROR_CODE_MOVEMENT_IN_MISSION_RUNNING = 320006,   /* 启动移动任务时，mission在运行    请等空闲之后，再试*/
    ERROR_CODE_MOVEMENT_SCREEN_TURNED_UP = 320007,   /* 触摸屏被翻起来了，不能启动移动任务    请将触摸屏放置成水平状态后，再试*/
    ERROR_CODE_MOVEMENT_FOLLOW_PATH_START_POSE_OFFSET = 320010,   /* 用户发送移动路径时，起始点偏离了agv的真实位置，阈值由main.manual_path_start_pose_check_threshold配置    请检测下发参数后，再试*/
    ERROR_CODE_MOVEMENT_FOLLOW_PATH_EXIST_ARC = 320011,   /* 用户发送移动路径时，路径中存在圆弧路径，圆弧路径已经被废弃    请检测下发参数后，再试*/
    ERROR_CODE_MOVEMENT_FOLLOW_PATH_START_ANGLE_OFFSET = 320012,   /* 用户发送路径时，起点的角度与当前src角度相差过大，阈值由main.manual_path_start_angle_check_threshold配置    请检测下发参数后，再试*/
    ERROR_CODE_MOVEMENT_FOLLOW_PATH_POSE_NOT_CONTINUOUS = 320013,   /* 用户发送移动路径，路径不连续，阈值值由main.manual_path_pose_continuous_check_threshold 配置    请检测下发参数后，再试*/
    ERROR_CODE_MOVEMENT_FOLLOW_PATH_ANGLE_NOT_CONTINUOUS = 320014,   /* 用户发送移动路径，角度不连续，阈值值由main.manual_path_angle_continuous_check_threshold 配置    请检测下发参数后，再试*/
    ERROR_CODE_MOVEMENT_IN_POWER_SAVE_MODE = 320020,   /* 处于低功耗模式，不能启动移动任务    请退出低功耗模式后，再试*/
    ERROR_CODE_MOVEMENT_BREAK_SWITCH_ON = 320021,   /* 处于解抱闸状态（电源开关处于第二档），不能启动移动任务    请取消解抱闸（电源开关打到第三档）后，再试*/
    ERROR_CODE_MOVEMENT_LASER_ERROR = 320022,   /* 雷达错误，不能启动移动任务    请等待雷达错误回复后，再试。比如解除低功耗模式时雷达会初始化，此期间会出现一段时间雷达错误*/
    ERROR_CODE_MOVEMENT_VSC_ERROR = 320023,   /* VSC错误，不能启动移动任务    重启*/
    ERROR_CODE_MOVEMENT_SRC_ERROR = 320024,   /* SRC错误，不能启动移动任务    重启*/
    ERROR_CODE_MOVEMENT_MOTOR1_ERROR = 320025,   /* 电机1错误，不能启动移动任务    等待电机错误回复后再试，如：解除急停后电机会初始化，此期间会出现一段时间电机错误*/
    ERROR_CODE_MOVEMENT_MOTOR2_ERROR = 320026,   /* 电机2错误，不能启动移动任务    等待电机错误回复后再试，如：解除急停后电机会初始化，此期间会出现一段时间电机错误*/
    ERROR_CODE_MOVEMENT_FAULT_EXIST = 320027,   /* 存在一些影响移动任务运行的故障，不能启动移动任务    1.修复故障后再试
2.处于调试阶段，可以设置inspection中的一些参数来忽略一些非严重的设备故障，正常后记得恢复这些参数。*/
    ERROR_CODE_MOVEMENT_BATTERY_CHARGING = 320028,   /* 正在充电，不能启动移动任务    停止充电后，再试*/
    ERROR_CODE_MOVEMENT_EAC_PROHIBIT = 320029,   /* EAC当前禁止启动移动任务，EAC认为当前启动移动任务会带来风险    请检查EAC异常(可查看EAC故障码)后，再试*/
    ERROR_CODE_MOVEMENT_ROTATE_IN_DISABLE_ROTATE_AREA = 320030,   /* 规划的路径在禁止旋转区域旋转    请检测规划出来的路径是否符合期望*/
    ERROR_CODE_MOVEMENT_PATHS_CHASSIS_NOT_SUPPORT = 320031,   /* 当前底盘不支持本次规划的路径    请检测地图路径是否编辑正确，并检查车辆的底盘是否配置正确(参数：src.base_type)*/
    ERROR_CODE_FIND_PATH_UNKNOWN_ERROR = 321000,   /* 搜索路径时未知错误    联系厂家处理*/
    ERROR_CODE_NAV_FIND_PATH_NO_STATION_IN_MAP = 321001,   /* 搜索路径失败，在地图中不存在该站点    请检测地图中是否存在需要导航到的站点，再试*/
    ERROR_CODE_NAV_FIND_PATH_NET_NAV_AND_FREE_NAV_BOTH_DISABLED = 321002,   /* 路网导航和自由导航都没有开启    请需要导航的模式后，再试*/
    ERROR_CODE_NET_NAV_START_POSE_NOT_ON_NET = 321005,   /* 路网导航，起点未在路径上    请将车辆移动到路网上后，再试*/
    ERROR_CODE_NET_NAV_DST_POSE_NOT_ON_NET = 321006,   /* 路网导航，终点未在路径上    请检测要导航到的终点是否在路网上后，再试*/
    ERROR_CODE_NET_NAV_NONE_STATION_AROUND_AGV = 321007,   /* 路网导航，设置了首先移动到最近的站点，但是车辆附近找不到对应的站点    请将车辆移动到站点后，再试*/
    ERROR_CODE_NET_NAV_FIND_PATH_NO_WAY = 321008,   /* 路网导航，没有路径到达目标位置    请确认导航的起点和终点之间是否有路后，再试*/
    ERROR_CODE_FREE_NAV_DST_POSE_UNWALKABLE = 321010,   /* 自由导航失败，目标站点不可到达    请确认导航的起点和终点之间是否有路后，再试*/
    ERROR_CODE_FREE_NAV_NO_WAY = 321011,   /* 自由导航失败，没找到合适的路径    请确认导航的起点和终点之间是否有路后，再试*/
    ERROR_CODE_AGV_NOT_ARRIVED_DEST = 321016,   /* agv没有导航到达目标位置，检测阈值由“main.movement_task_arrive_point_check_threshold”配置    请检测是否正在到达了目标位置*/
    ERROR_CODE_NAV_TO_NEAREST_FIRST_BUT_FREE_NAV_DISABLED = 321017,   /* 路网导航时，设置了首先移动到最近的站点，但是没有开启自由导航    请启动自由导航之后再试*/
    ERROR_CODE_NAV_TO_NEAREST_FIRST_BUT_FREE_NAV_NO_WAY = 321018,   /* 路网导航时，设置了首先移动到最近的站点，但是自由导航无法到达    请检查路径是否真的可达*/
    ERROR_CODE_FEATURE_RECOGNIZE_DEPARTURE = 321019,   /* 站点特征对接时，特征偏差过大    请检查特征位置是否真的可达*/
    ERROR_CODE_FEATURE_RECOGNIZE_NOT_DISCERN = 321020,   /* 站点特征对接时，未识别到指定特征    请检查特征是否真的可识别*/
    ERROR_CODE_FEATURE_RECOGNIZE_NOT_MATCH = 321021,   /* 站点特征对接时，识别到特征但不匹配    请检查特征是否真的可识别*/
    ERROR_CODE_CANCEL_MOVEMENT_TASK_NOT_RUNNING = 330001,   /* 取消移动任务时，移动任务并没有启动    没有移动任务，无需取消*/
    ERROR_CODE_SET_CHECKPOINT_TASK_NOT_RUNNING = 330010,   /* 设置关卡时，移动任务并没有启动    请确认是否有移动任务运行后，再试*/
    ERROR_CODE_SET_CHECKPOINT_TASK_TYPE_NOT_FOLLOW_PATH = 330011,   /* 设置关卡时，移动任务不是跟随路径任务    请移动任务是否是跟随路径任务，再试*/
    ERROR_CODE_CHECKPOINT_LESS_THAN_OR_EQUAL_CURRENT = 330012,   /* 设置关卡时，关卡的标号小于等于当前当前的关卡编号（保留）    请确认当前关卡编号，再试*/
    ERROR_CODE_PATH_REPLACE_MOVEMENT_TASK_NOT_RUNNING = 330020,   /* 替换路径时移动任务没有启动    移动任务可能已经结束了，不能继续替换路径，请启动新的移动任务*/
    ERROR_CODE_PATH_REPLACE_MOVEMENT_TASK_TYPE_NOT_MOVE_FOLLOW_PATH = 330021,   /* 只有MOVE_FOLLOW_PATH的移动任务才允许替换路径    请将移动任务类型改为MOVE_FOLLOW_PATH，然后再替换路径*/
    ERROR_CODE_PATH_REPLACE_MOVEMENT_TASK_NO_MISMATCH = 330023,   /* 替换路径时，task no不匹配    请检查task no，然后再替换路径*/
    ERROR_CODE_PATH_REPLACE_CURRENT_PATH_NUM_IS_NOT_SINGLE = 330024,   /* 替换路径时，当前移动任务的路径不止一条    只有当是单条路径的时候才允许替换路径*/
    ERROR_CODE_PATH_REPLACE_DEST_POSE_PAST = 330025,   /* 替换路径时，目标站点已经走过了    请重新规划一条路径去目标站点*/
    ERROR_CODE_PATH_REPLACE_PATHS_ARG_COUNT_IS_NOT_ONE = 330026,   /* 替换路径时，paths的size不为1    请检测参数后，再试*/
    ERROR_CODE_PATH_REPLACE_INCONSISTENT_PATHS_SLOP = 330027,   /* 替换路径时，paths路径的斜率不一致    请检测路径的斜率是否一致，再试*/
    ERROR_CODE_PATH_REPLACE_START_POINT_CHANGED = 330028,   /* 替换路径时，起始点不一致    请起始点是否一致，再试*/
    ERROR_CODE_ACTION_IN_EMERGENCY = 340001,   /* 车辆处于急停状态，不能启动动作任务    解除急停后，再试*/
    ERROR_CODE_ACTION_PRE_TASK_RUNNING = 340002,   /* 上一个动作任务正在执行，不能启动新的动作任务    等待上一个任务结束后，再试*/
    ERROR_CODE_ACTION_IN_POWER_SAVE_MODE = 340003,   /* 处于低功耗模式，不能启动动作任务    取消低功耗模式后，再试*/
    ERROR_CODE_ACTION_IN_MISSION_RUNNING = 340006,   /* mission在运行的时候，不能启动动作任务    请等mission结束后，再试*/
    ERROR_CODE_ACTION_SCREEN_TURNED_UP = 340007,   /* 触摸屏被翻起来了，不能启动动作任务    请将触摸屏放置成水平状态后，再试*/
    ERROR_CODE_ACTION_BREAK_SWITCH_ON = 340008,   /* 处于解抱闸状态（电源开关处于第二档），不能启动动作任务    请取消解抱闸（电源开关打到第三档）后，再试*/
    ERROR_CODE_ACTION_RESPONSE_TIMEOUT = 340010,   /* 动作执行机构回复超时    请检测动作执行机构是否正常后，再试*/
    ERROR_CODE_ACTION_RESPONSE_INCORRECT = 340011,   /* 动作执行机构回复出错    请检测动作执行机构是否正常后，再试*/
    ERROR_CODE_ACTION_SYSTEM_BUSY = 340012,   /* 动作执行机构系统繁忙    请等执行机构空闲后，再试*/
    ERROR_CODE_ACTION_ID_NOT_SUPPORT = 340013,   /* action id 不支持    请检测下发参数后，再试*/
    ERROR_CODE_ACTION_PARAM_0_NOT_SUPPORT = 340014,   /* action param 0 不支持    请检测下发参数后，再试*/
    ERROR_CODE_ACTION_PARAM_1_NOT_SUPPORT = 340015,   /* action param 1 不支持    请检测下发参数后，再试*/
    ERROR_CODE_ACTION_EAC_DISABLED = 340016,   /* EAC模块没有开启    请检测EAC模块是否接上了后，再试*/
    ERROR_CODE_ACTION_EAC_NOT_IDLE = 340017,   /* EAC未处于空闲状态，不能启动新的动作任务    请确认EAC是否处于空闲状态*/
    ERROR_CODE_ACTION_EAC_IN_ERROR_STATE = 340018,   /* EAC处于故障状态，不能启动新的动作任务    人工清除物理故障后，点击Matrix上的“尝试解除故障”按钮，解除EAC故障后再尝试*/
    ERROR_CODE_ACTION_EAC_PAUSE_FAILED = 340019,   /* EAC暂停失败    请确认该动作是否支持暂停，再试*/
    ERROR_CODE_ACTION_EAC_CONTINUE_FAILED = 340020,   /* EAC继续失败    请确认EAC是否正常，再试*/
    ERROR_CODE_ACTION_FAULT_EXIST = 340027,   /* 存在一些影响动作任务运行的故障，不能启动动作任务    1.修复故障后再试
2.处于调试阶段，可以设置inspection中的一些参数来忽略一些非严重的设备故障，正常后记得恢复这些参数。*/
    ERROR_CODE_ACTION_DOWN_CANMER_RECITIFY_DISABLED = 340701,   /* 下视二维码矫正功能没有开启    请将参数“main.enable_pgv_rectify”设置为True后，再试*/
    ERROR_CODE_ACTION_DOWN_CANMER_RECITIFY_CURRENT_STATION_IS_NONE = 340702,   /* 下视二维码矫正时，当前站点为0    请确定矫正位置是否正确后，再试*/
    ERROR_CODE_ACTION_DOWN_CANMER_RECITIFY_CURRENT_STATION_DMCODE_IS_NONE = 340703,   /* 当前站点没有绑定下视二维码    请将当前站点二维码偏差设置好，再试*/
    ERROR_CODE_ACTION_FORK_LIFT_NOT_REACH = 340801,   /* 空叉升降失败    请检查空货叉升降是否正常*/
    ERROR_CODE_ACTION_GOODS_NOT_DETECTED = 340802,   /* 货架上未检测到货物    请检查货架上是否有货物*/
    ERROR_CODE_ACTION_GOODS_POS_YAW_DEVIATE = 340803,   /* 3D视觉相机检测到货物摆放角度过偏    请检查货物摆放角度是否过偏并摆正*/
    ERROR_CODE_ACTION_GOODS_POS_X_DEVIATE_FORWARD = 340804,   /* 3D视觉相机检测到货物X方向太靠前    请检查货物摆放X方向是否太靠前并摆正*/
    ERROR_CODE_ACTION_GOODS_POS_X_DEVIATE_BACK = 340805,   /* 3D视觉相机检测到货物X方向太靠后    请检查货物摆放X方向是否太靠后并摆正*/
    ERROR_CODE_ACTION_GOODS_POS_Y_DEVIATE = 340806,   /* 3D视觉相机检测到货物摆放Y方向过偏    请检查货物摆放Y方向是否过偏并摆正*/
    ERROR_CODE_ACTION_LOAD_NO_DOCKING = 340807,   /* 货叉取货路径对接失败    请排查失败原因*/
    ERROR_CODE_ACTION_FORK_FAIL_PICKUP_GOODS = 340808,   /* 载货叉臂上升失败    请检查载货叉臂上升功能是否正常*/
    ERROR_CODE_ACTION_BACKFORK_MOVE_NOT_REACH = 340809,   /* 取货退出路径导航失败    请排查失败原因*/
    ERROR_CODE_ACTION_GOODS_DETECTED = 340810,   /* 卸货位上有货物    请检查当前卸货位是否有货物*/
    ERROR_CODE_ACTION_UNLOAD_MOVE_NOT_REACH = 340811,   /* 卸货路径导航失败    请排查失败原因*/
    ERROR_CODE_ACTION_LOAD_PALLET_BOUNCE = 340812,   /* 载货时到位开关未触发    请排查失败原因*/
    ERROR_CODE_ACTION_FORK_FAIL_PICKDOWN_GOODS = 340813,   /* 卸货叉臂下降失败    请检查卸货叉臂下降功能是否正常*/
    ERROR_CODE_GET_SCAN_CODE_ACTION_PARAM_INVALID = 407000,   /* 参数无效    请检测任务参数后，再试*/
    ERROR_CODE_SCAN_ERROR = 407001,   /* 扫码出错或结果为空    请检测车辆是否能扫描到条码之后，再试*/
    ERROR_CODE_CHARGE_PARAM_INVALID = 407800,   /* 充电参数无效    请检测任务参数后，再试*/
    ERROR_CODE_CHARGE_TO_PERCENT_FAILED_NO_CHARGE_DOCK = 407802,   /* 启动自动充电到指定百分比任务失败    请检测车辆是否和充电桩是否连接上了后，再试*/
    ERROR_CODE_CHARGE_TO_PERCENT_FAILED_DETACH_FROM_CHARGE_DOCK = 407803,   /* 充电过程中车辆和充电桩断开了    请检测车辆是否和充电桩是否连接上了后，再试*/
    ERROR_CODE_CHARGE_FAILED_NO_CHARGE_DOCK = 407805,   /* 充电到某个值任务失败    请检测车辆是否和充电桩是否连接上了后，再试*/
    ERROR_CODE_CHARGE_FAILED_CURRENT_MANUAL_CONTROL = 407806,   /* 启动充电失败，当前处于手动控制状态    请关闭手动控制后，再试*/
    ERROR_CODE_CHARGE_FAILED_MOVEMENT_RUNNING = 407810,   /* 启动自动充电失败，当前移动任务在运行，启动充电可能会产生火花    请等待任务结束后，再试*/
    ERROR_CODE_STOP_CHARGE_FAILED = 407820,   /* 结束充电失败，可能当前用的是手动充电器充电    请将手动充电器安全移除即可*/
    ERROR_CODE_RFID_NOT_ENABLED = 413201,   /* RFID未开启    请将参数“device.enable_rfid“修改为True，再试*/
    ERROR_CODE_RFID_GET_NONE = 413202,   /* 未获取到RFID数据    请检测车辆是否能扫描到RFID之后，再试*/
    ERROR_CODE_UP_SVC100_NOT_OPEN = 413301,   /* 上视摄像头未开启    开启上视摄像头*/
    ERROR_CODE_DOWN_SVC100_NOT_OPEN = 413302,   /* 下视摄像头未开启    开启下视摄像头*/
    ERROR_CODE_WAIT_UP_DMCODE_OVERTIME = 413310,   /* 指定时间内上视摄像头没有检测到二维码    请将二维码放到上视摄像头可以扫描到的位置，后再试。*/
    ERROR_CODE_WAIT_DOWN_DMCODE_OVERTIME = 413311,   /* 指定时间内下视视摄像头没有检测到二维码    请将二维码放到下视视摄像头可以扫描到的位置，后再试。*/
    ERROR_CODE_WAIT_GPIO_OVERTIME = 413401,   /* 指定时间内没有放行    请在指定时间内进行放行操作*/
    ERROR_CODE_TARGET_DETECT_OVERTIME = 413501,   /* 目标检测超时    请排查失败原因*/
    ERROR_CODE_MAP_SWITCHING_IN_START_LOCATION = 550001,   /* 切换地图时，当前正在定位    请稍后再试*/
    ERROR_CODE_MAP_SWITCHING_IN_PROCESSING = 550002,   /* 切换地图时，当前正在切换地图，且两次切换的地图还不一致    请稍后再试*/
    ERROR_CODE_MAP_SWITCHING_MOVEMENT_RUNNING = 550003,   /* 切换地图时，移动任务正在运行    请等移动任务结束后再试*/
    ERROR_CODE_MAP_SWITCHING_MAP_LOAD_ERROR = 550005,   /* 切换地图时，当前地图不存在或者是当前地图数据被破坏    请确认当前地图名是否正确，再试*/
    ERROR_CODE_MAP_SWITCHING_STATION_NOT_EXIST = 550006,   /* 切换地图时，通过站点定位，但是该站点不存在    确保车辆在站点上，才能通过站点切换地图定位*/
    ERROR_CODE_MISSION_IN_EMERGENCY = 660001,   /* 启动任务时，车辆处于急停状态    解除急停后，再试*/
    ERROR_CODE_MISSION_NO_LOCATION = 660002,   /* 启动任务时，车辆不处于定位状态    定位后，再试*/
    ERROR_CODE_MISSION_ID_NOT_EXIST = 660005,   /* 启动任务时，任务Id不存在    请检测下发参数后，再试*/
    ERROR_CODE_MISSION_ENQUEUE_TIMEOUT = 660006,   /* 启动任务时，加入队列超时    再试*/

};

}  // namespace core
}  // namespace sros

#endif  // ERROR_CODE_H_

