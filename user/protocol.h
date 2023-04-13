#ifndef __PROTOCOL_H
#define __PROTOCOL_H

/**
 * @file    protocol.h
 * @note    本文件由 Node_Bridge 生成, 请勿直接修改
 *
 * @note    Node     command_id
 *          裁判系统  0x0000 - 0x03FF
 *          上位机    0x0400 - 0x04FF
 *          板间通讯  0x0500 - 0x05FF
 *          裁判系统  0xF100 - 0xF1FF
 *          车间通讯  0xF200 - 0xF2FF
 * @version judge.yml v1.0
 * @version host.yml v0.1
 * @version user.yml v0.1
 */

/**********************************************************************
 *                              结构体
 **********************************************************************/

typedef struct {
    uint8_t  graphic_name[3];
    uint32_t operate_tpye : 3;
    uint32_t graphic_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t start_angle : 9;
    uint32_t end_angle : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t radius : 10;
    uint32_t end_x : 11;
    uint32_t end_y : 11;
} GraphicData_Type;

typedef struct {
    int32_t debug1;
    int32_t debug2;
    int32_t debug3;
    int32_t debug4;
    int32_t debug5;
    int32_t debug6;
    int32_t debug7;
    int32_t debug8;
} DebugData_Type;

/**********************************************************************
 *                              协议列表
 **********************************************************************/

typedef union {
    struct {
        uint8_t  game_type : 4;
        uint8_t  game_progress : 4;
        uint16_t stage_remain_time;
        uint64_t SyncTimeStamp;
    };
    struct {
        uint8_t data[11];
    };
} game_status_t;

typedef union {
    struct {
        uint8_t winner;
    };
    struct {
        uint8_t data[1];
    };
} game_result_t;

typedef union {
    struct {
        uint16_t red_1_robot_HP;
        uint16_t red_2_robot_HP;
        uint16_t red_3_robot_HP;
        uint16_t red_4_robot_HP;
        uint16_t red_5_robot_HP;
        uint16_t red_7_robot_HP;
        uint16_t red_outpost_HP;
        uint16_t red_base_HP;
        uint16_t blue_1_robot_HP;
        uint16_t blue_2_robot_HP;
        uint16_t blue_3_robot_HP;
        uint16_t blue_4_robot_HP;
        uint16_t blue_5_robot_HP;
        uint16_t blue_7_robot_HP;
        uint16_t blue_outpost_HP;
        uint16_t blue_base_HP;
    };
    struct {
        uint8_t data[32];
    };
} game_robot_HP_t;

typedef union {
    struct {
        uint8_t  dart_belong;
        uint16_t stage_remaining_time;
    };
    struct {
        uint8_t data[3];
    };
} dart_status_t;

typedef union {
    struct {
        uint32_t event_type;
    };
    struct {
        uint8_t data[4];
    };
} event_data_t;

typedef union {
    struct {
        uint8_t supply_projectile_id;
        uint8_t supply_robot_id;
        uint8_t supply_projectile_step;
        uint8_t supply_projectile_num;
    };
    struct {
        uint8_t data[4];
    };
} supply_projectile_action_t;

typedef union {
    struct {
        uint8_t level;
        uint8_t foul_robot_id;
    };
    struct {
        uint8_t data[2];
    };
} referee_warning_t;

typedef union {
    struct {
        uint8_t dart_remaining_time;
    };
    struct {
        uint8_t data[1];
    };
} dart_remaining_time_t;

typedef union {
    struct {
        uint8_t  robot_id;
        uint8_t  robot_level;
        uint16_t remain_HP;
        uint16_t max_HP;
        uint16_t shooter_id1_17mm_cooling_rate;
        uint16_t shooter_id1_17mm_cooling_limit;
        uint16_t shooter_id1_17mm_speed_limit;
        uint16_t shooter_id2_17mm_cooling_rate;
        uint16_t shooter_id2_17mm_cooling_limit;
        uint16_t shooter_id2_17mm_speed_limit;
        uint16_t shooter_id1_42mm_cooling_rate;
        uint16_t shooter_id1_42mm_cooling_limit;
        uint16_t shooter_id1_42mm_speed_limit;
        uint16_t chassis_power_limit;
        uint8_t  mains_power_gimbal_output : 1;
        uint8_t  mains_power_chassis_output : 1;
        uint8_t  mains_power_shooter_output : 1;
    };
    struct {
        uint8_t data[27];
    };
} game_robot_status_t;

typedef union {
    struct {
        uint16_t chassis_volt;
        uint16_t chassis_current;
        float    chassis_power;
        uint16_t chassis_power_buffer;
        uint16_t shooter_id1_17mm_cooling_heat;
        uint16_t shooter_id2_17mm_cooling_heat;
        uint16_t shooter_id1_42mm_cooling_heat;
    };
    struct {
        uint8_t data[16];
    };
} power_heat_data_t;

typedef union {
    struct {
        float x;
        float y;
        float z;
        float yaw;
    };
    struct {
        uint8_t data[16];
    };
} game_robot_pos_t;

typedef union {
    struct {
        uint8_t power_rune_buff;
    };
    struct {
        uint8_t data[1];
    };
} buff_info_t;

typedef union {
    struct {
        uint8_t attack_time;
    };
    struct {
        uint8_t data[1];
    };
} aerial_robot_energy_t;

typedef union {
    struct {
        uint8_t armor_id : 4;
        uint8_t hurt_type : 4;
    };
    struct {
        uint8_t data[1];
    };
} robot_hurt_t;

typedef union {
    struct {
        uint8_t bullet_type;
        uint8_t shooter_id;
        uint8_t bullet_freq;
        float   bullet_speed;
    };
    struct {
        uint8_t data[7];
    };
} shoot_data_t;

typedef union {
    struct {
        uint16_t bullet_remaining_num_17mm;
        uint16_t bullet_remaining_num_42mm;
        uint16_t coin_remaining_num;
    };
    struct {
        uint8_t data[6];
    };
} bullet_remaining_t;

typedef union {
    struct {
        uint32_t rfid_status;
    };
    struct {
        uint8_t data[4];
    };
} rfid_status_t;

typedef union {
    struct {
        uint8_t  dart_launch_opening_status;
        uint8_t  dart_attack_target;
        uint16_t target_change_time;
        uint8_t  first_dart_speed;
        uint8_t  second_dart_speed;
        uint8_t  third_dart_speed;
        uint8_t  fourth_dart_speed;
        uint16_t last_dart_launch_time;
        uint16_t operate_launch_cmd_time;
    };
    struct {
        uint8_t data[12];
    };
} dart_client_cmd_t;

typedef union {
    struct {
        uint16_t data_cmd_id;
        uint16_t send_id;
        uint16_t receiver_id;
        uint8_t  operate_tpye;
        uint8_t  layer;
    };
    struct {
        uint8_t data[8];
    };
} client_custom_graphic_delete_t;

typedef union {
    struct {
        uint16_t         data_cmd_id;
        uint16_t         send_id;
        uint16_t         receiver_id;
        GraphicData_Type grapic_data_struct;
    };
    struct {
        uint8_t data[21];
    };
} client_custom_graphic_single_t;

typedef union {
    struct {
        uint16_t         data_cmd_id;
        uint16_t         send_id;
        uint16_t         receiver_id;
        GraphicData_Type grapic_data_struct[2];
    };
    struct {
        uint8_t data[36];
    };
} client_custom_graphic_double_t;

typedef union {
    struct {
        uint16_t         data_cmd_id;
        uint16_t         send_id;
        uint16_t         receiver_id;
        GraphicData_Type grapic_data_struct[5];
    };
    struct {
        uint8_t data[81];
    };
} client_custom_graphic_five_t;

typedef union {
    struct {
        uint16_t         data_cmd_id;
        uint16_t         send_id;
        uint16_t         receiver_id;
        GraphicData_Type grapic_data_struct;
        uint8_t          data[30];
    };
    struct {
        uint8_t data[51];
    };
} client_custom_character_t;

typedef union {
    struct {
        uint16_t         data_cmd_id;
        uint16_t         send_id;
        uint16_t         receiver_id;
        GraphicData_Type grapic_data_struct[7];
    };
    struct {
        uint8_t data[111];
    };
} client_custom_graphic_seven_t;

typedef union {
    struct {
        DebugData_Type debugData;
    };
    struct {
        uint8_t data[32];
    };
} debug_info_t;

typedef union {
    struct {
        uint16_t code;
        char     text[22];
    };
    struct {
        uint8_t data[24];
    };
} error_info_t;

typedef union {
    struct {};
    struct {
        uint8_t data[0];
    };
} heartbeat_t;

typedef union {
    struct {
        float   yaw_angle_diff;
        float   pitch_angle_diff;
        uint8_t fire;
    };
    struct {
        uint8_t data[9];
    };
} autoaim_data_t;

typedef union {
    struct {
        float vx;
        float vy;
        float vw;
    };
    struct {
        uint8_t data[12];
    };
} chassis_data_t;

typedef union {
    struct {
        float yaw;
        float pitch;
        float roll;
    };
    struct {
        uint8_t data[12];
    };
} gyroscope_data_t;

typedef union {
    struct {
        uint8_t dbusBuffer[18];
    };
    struct {
        uint8_t data[18];
    };
} dbus_data_t;

typedef union {
    struct {
        uint16_t data_cmd_id;
        uint16_t send_id;
        uint16_t receiver_id;
        int32_t  data1;
        int32_t  data2;
        int32_t  data3;
        int32_t  data4;
    };
    struct {
        uint8_t data[22];
    };
} robot_interactive_data_t;

typedef union {
    struct {
        int32_t data1;
        int32_t data2;
        int32_t data3;
        int32_t data4;
    };
    struct {
        uint8_t data[16];
    };
} board_interactive_data_t;

typedef union {
		struct {
				uint8_t graphic_name[3];
		};
		struct {
uint32_t operate_type:3;
uint32_t graphic_type:3;
uint32_t layer:4;
uint32_t color:4;
uint32_t start_angle:9;
uint32_t end_angle:9;
uint32_t width:10;
uint32_t start_x:11;
uint32_t start_y:11;
uint32_t radius:10;
uint32_t end_x:11;
uint32_t end_y:11;
		};
} graphic_data_struct_t;

/**********************************************************************
 *                              协议信息
 **********************************************************************/

#define PROTOCOL_INFO_LIST                                                                                                                                     \
    {                                                                                                                                                          \
        {0X0001, 11, 1}, {0X0002, 1, 1}, {0X0003, 32, 1}, {0X0004, 3, 1}, {0X0101, 4, 1}, {0X0102, 4, 1}, {0X0104, 2, 1}, {0X0105, 1, 1}, {0X0201, 27, 1},     \
            {0X0202, 16, 1}, {0X0203, 16, 1}, {0X0204, 1, 1}, {0X0205, 1, 1}, {0X0206, 1, 1}, {0X0207, 7, 1}, {0X0208, 6, 1}, {0X0209, 4, 1}, {0X020A, 12, 1}, \
            {0XF100, 8, 0}, {0XF101, 21, 0}, {0XF102, 36, 0}, {0XF103, 81, 0}, {0XF110, 51, 0}, {0XF104, 111, 0}, {0X1024, 32, 0}, {0X6666, 24, 0},            \
            {0X0120, 0, 1}, {0X0401, 9, 1}, {0X0402, 12, 1}, {0X0403, 12, 0}, {0X0404, 18, 0}, {0XF301, 22, 1}, {0X0501, 16, 1}, {                             \
            0X0502, 16, 1                                                                                                                                      \
        }                                                                                                                                                      \
    }

/**********************************************************************
 *                              协议接口
 **********************************************************************/

typedef union {
    struct {
        game_status_t                  gameStatus;                  // 0X0001 比赛状态数据
        game_result_t                  gameResult;                  // 0X0002 比赛结果的数据
        game_robot_HP_t                healthPoint;                 // 0X0003 机器人血量数据
        dart_status_t                  dartStatus;                  // 0X0004 飞镖发射状态
        event_data_t                   eventData;                   // 0X0101 场地事件数据
        supply_projectile_action_t     supplyProjectileaction;      // 0X0102 补给站动作标识
        referee_warning_t              refereeWarning;              // 0X0104 裁判警告信息
        dart_remaining_time_t          dartRemainingtime;           // 0X0105 飞镖发射口倒计
        game_robot_status_t            gameRobotstatus;             // 0X0201 比赛机器人状态
        power_heat_data_t              powerHeatData;               // 0X0202 实时功率热量数据
        game_robot_pos_t               gameRobotpos;                // 0X0203 机器人位置
        buff_info_t                    buffInfo;                    // 0X0204 机器人增益
        aerial_robot_energy_t          aerialRobotenergy;           // 0X0205 空中机器人能量状态
        robot_hurt_t                   robotHurt;                   // 0X0206 伤害状态
        shoot_data_t                   shootData;                   // 0X0207 实时射击信息
        bullet_remaining_t             bulletRemaining;             // 0X0208 子弹剩余发射数
        rfid_status_t                  rfidStatus;                  // 0X0209 机器人RFID状态
        dart_client_cmd_t              dartClientCmds;              // 0X020A 飞镖机器人客户端指令数据
        client_custom_graphic_delete_t clientCustomGraphicDelete;   // 0XF100 客户端删除图形
        client_custom_graphic_single_t client_custom_graphicSingle; // 0XF101 客户端绘制一个图形
        client_custom_graphic_double_t clientCustomGraphicDouble;   // 0XF102 客户端绘制两个图形
        client_custom_graphic_five_t   clientCustomGraphicFive;     // 0XF103 客户端绘制五个图形
        client_custom_character_t      clientCustomCharacter;       // 0XF110 客户端绘制字符
        client_custom_graphic_seven_t  clientCustomGraphicSeven;    // 0XF104 客户端绘制七个图形
        debug_info_t                   debugInfo;                   // 0X1024 调试信息
        error_info_t                   errorInfo;                   // 0X6666 报错信息
        heartbeat_t                    heartbeat;                   // 0X0120 心跳包
        autoaim_data_t                 autoaimData;                 // 0X0401 云台控制
        chassis_data_t                 chassisData;                 // 0X0402 底盘控制
        gyroscope_data_t               gyroscopeData;               // 0X0403 陀螺仪数据
        dbus_data_t                    dbusData;                    // 0X0404 遥控器及键鼠数据
        robot_interactive_data_t       robotCommunication;          // 0XF301 学生机器人间通信
        board_interactive_data_t       boardAlpha;                  // 0X0501 主控板间通信
        board_interactive_data_t       boardBeta;                   // 0X0502 主控板间通信
    };
    struct {
        uint8_t data[618];
    };
} ProtocolData_Type;

#endif