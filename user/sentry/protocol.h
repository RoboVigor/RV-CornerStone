#ifndef __PROTOCOL_H
#define __PROTOCOL_H
/**
 * @file    protocol.h
 * @note    Node     command_id
 *          裁判系统  0x0000 - 0x03FF
 *          上位机    0x0400 - 0x04FF
 *          板间通讯  0x0500 - 0x05FF
 *          裁判系统  0xF100 - 0xF1FF
 *          车间通讯  0xF200 - 0xF2FF
 * @version 0.1
 */

/**
 * @brief   结构体
 */
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
} graphic_data_struct_t;

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

/**
 * @brief   协议列表
 */

typedef struct {
    union {
        struct {
            uint8_t  robot_id;
            uint8_t  robot_level;
            uint16_t remain_HP;
            uint16_t max_HP;
            uint16_t shooter_heat0_cooling_rate;
            uint16_t shooter_heat0_cooling_limit;
            uint16_t shooter_heat1_cooling_rate;
            uint16_t shooter_heat1_cooling_limit;
            uint8_t  shooter_heat0_speed_limit;
            uint8_t  shooter_heat1_speed_limit;
            uint8_t  max_chassis_power;
            uint8_t  mains_power_gimbal_output : 1;
            uint8_t  mains_power_chassis_output : 1;
            uint8_t  mains_power_shooter_output : 1;
        };
        struct {
            uint8_t data[18];
        };
    };
} ext_game_robot_state_t;

typedef struct {
    union {
        struct {
            uint16_t chassis_volt;
            uint16_t chassis_current;
            float    chassis_power;
            uint16_t chassis_power_buffer;
            uint16_t shooter_heat0;
            uint16_t shooter_heat1;
            uint16_t mobile_shooter_heat2;
        };
        struct {
            uint8_t data[16];
        };
    };
} ext_power_heat_data_t;

typedef struct {
    union {
        struct {
            uint16_t energy_point;
            uint8_t  attack_time;
        };
        struct {
            uint8_t data[3];
        };
    };
} aerial_robot_energy_t;

typedef struct {
    union {
        struct {
            uint8_t armor_id : 4;
            uint8_t hurt_type : 4;
        };
        struct {
            uint8_t data[1];
        };
    };
} ext_robot_hurt_t;

typedef struct {
    union {
        struct {
            uint8_t bullet_type;
            uint8_t bullet_freq;
            float   bullet_speed;
        };
        struct {
            uint8_t data[6];
        };
    };
} ext_shoot_data_t;

typedef struct {
    union {
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
    };
} client_custom_graphic_delete_t;

typedef struct {
    union {
        struct {
            uint16_t              data_cmd_id;
            uint16_t              send_id;
            uint16_t              receiver_id;
            graphic_data_struct_t grapic_data_struct;
        };
        struct {
            uint8_t data[21];
        };
    };
} client_custom_graphic_single_t;

typedef struct {
    union {
        struct {
            uint16_t              data_cmd_id;
            uint16_t              send_id;
            uint16_t              receiver_id;
            graphic_data_struct_t grapic_data_struct[2];
        };
        struct {
            uint8_t data[36];
        };
    };
} client_custom_graphic_double_t;

typedef struct {
    union {
        struct {
            uint16_t              data_cmd_id;
            uint16_t              send_id;
            uint16_t              receiver_id;
            graphic_data_struct_t grapic_data_struct[5];
        };
        struct {
            uint8_t data[81];
        };
    };
} client_custom_graphic_five_t;

typedef struct {
    union {
        struct {
            uint16_t              data_cmd_id;
            uint16_t              send_id;
            uint16_t              receiver_id;
            graphic_data_struct_t grapic_data_struct[7];
        };
        struct {
            uint8_t data[111];
        };
    };
} client_custom_graphic_seven_t;

typedef struct {
    union {
        struct {
            uint16_t              data_cmd_id;
            uint16_t              send_id;
            uint16_t              receiver_id;
            graphic_data_struct_t grapic_data_struct;
            uint8_t               text[30];
        };
        struct {
            uint8_t data[51];
        };
    };
} client_custom_character_t;

typedef struct {
    union {
        struct {
            float   yaw_angle_diff;
            float   pitch_angle_diff;
            uint8_t biu_biu_state;
        };
        struct {
            uint8_t data[9];
        };
    };
} ext_gimbal_aim_data_t;

typedef struct {
    union {
        struct {
            float yaw;
            float pitch;
            float roll;
            float yawoffset;
        };
        struct {
            uint8_t data[16];
        };
    };
} board_Down_Gyroscope_data_t;

typedef struct {
    union {
        struct {
            int16_t gx;
            int16_t gy;
            int16_t gz;
        };
        struct {
            uint8_t data[6];
        };
    };
} board_Down_Imu_data_t;

typedef struct {
    union {
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
    };
} robot_interactive_data_t;

typedef struct {
    union {
        struct {
            uint16_t code;
            char     text[21];
        };
        struct {
            uint8_t data[23];
        };
    };
} error_data_t;

typedef struct {
    union {
        struct {
            DebugData_Type debugData;
        };
        struct {
            uint8_t data[32];
        };
    };
} DebugInfo_Type;

typedef struct {
    union {
        struct {
            uint16_t code;
            char     text[21];
        };
        struct {
            uint8_t data[23];
        };
    };
} ErrorInfo_Type;

typedef struct {
} Heartbeat_Type;

#define PROTOCOL_INFO_LIST                                                                                                                                     \
    {{0x0201, 18, 1},                                                                                                                                          \
     {0x202, 16, 1},                                                                                                                                           \
     {0x205, 3, 1},                                                                                                                                            \
     {0x206, 1, 1},                                                                                                                                            \
     {0x207, 6, 1},                                                                                                                                            \
     {0xF100, 8, 1},                                                                                                                                           \
     {0xF101, 21, 1},                                                                                                                                          \
     {0xF102, 36, 1},                                                                                                                                          \
     {0xF103, 81, 1},                                                                                                                                          \
     {0xF104, 111, 1},                                                                                                                                         \
     {0xF110, 51, 1},                                                                                                                                          \
     {0x0401, 9, 1},                                                                                                                                           \
     {0x0501, 16, 1},                                                                                                                                          \
     {0x0502, 6, 1},                                                                                                                                           \
     {0xF201, 22, 1},                                                                                                                                          \
     {0x120, 0, 1},                                                                                                                                            \
     {0x1024, 32, 1},                                                                                                                                          \
     {0x6666, 23, 1}};

/**
 * @brief   协议接口
 */

typedef union {
    struct {
        ext_game_robot_state_t         robotState;             // 状态数据
        ext_power_heat_data_t          powerHeatData;          // 热量数据
        aerial_robot_energy_t          aerialRobotEnergy;      // 空中机器人数据
        ext_robot_hurt_t               robotHurt;              // 伤害数据
        ext_shoot_data_t               shootData;              // 射击数据
        client_custom_graphic_delete_t clientGraphicDelete;    // 客户端自定义删除图形
        client_custom_graphic_single_t clientGraphicSingle;    // 客户端自定义绘制一个图形
        client_custom_graphic_double_t clientGraphicDouble;    // 客户端自定义绘制两个图形
        client_custom_graphic_five_t   clientGraphicFive;      // 客户端自定义绘制五个图形
        client_custom_graphic_seven_t  clientGraphicSeven;     // 客户端自定义绘制七个图形
        client_custom_character_t      clientGraphicCharacter; // 客户端自定义绘制字符
        ext_gimbal_aim_data_t          autoaimData;            // 视觉自瞄数据
        board_Down_Gyroscope_data_t    boardDownGyroscopeData; // 下云台陀螺仪系统数据
        board_Down_Imu_data_t          boardDownImuData;       // 下云台惯性测量单元数据
        robot_interactive_data_t       robotCommunication;     // 车间通讯测试
        Heartbeat_Type                 heartbeat;              // 心跳包
        DebugInfo_Type                 debugInfo;              // 调试信息
        ErrorInfo_Type                 errorInfo;              // 报错信息
    };
    struct {
        uint8_t data[460];
    };
} ProtocolData_Type;

#endif