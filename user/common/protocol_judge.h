/**
 * @brief   裁判系统通讯协议
 * @note    id: 0x0000 - 0x03FF
 * @version 1.0
 */

/**
 * @brief   协议数据段长度
 * @note    单位 bit
 */
#define PROTOCOL_PACK_LENGTH_0001 3
#define PROTOCOL_PACK_LENGTH_0002 1
#define PROTOCOL_PACK_LENGTH_0003 32
#define PROTOCOL_PACK_LENGTH_0004 3
#define PROTOCOL_PACK_LENGTH_0005 3
#define PROTOCOL_PACK_LENGTH_0101 4
#define PROTOCOL_PACK_LENGTH_0102 4
#define PROTOCOL_PACK_LENGTH_0104 2
#define PROTOCOL_PACK_LENGTH_0105 1
#define PROTOCOL_PACK_LENGTH_0201 18
#define PROTOCOL_PACK_LENGTH_0202 16
#define PROTOCOL_PACK_LENGTH_0203 16
#define PROTOCOL_PACK_LENGTH_0204 1
#define PROTOCOL_PACK_LENGTH_0205 3
#define PROTOCOL_PACK_LENGTH_0206 1
#define PROTOCOL_PACK_LENGTH_0207 6
#define PROTOCOL_PACK_LENGTH_0208 2
#define PROTOCOL_PACK_LENGTH_0209 4
#define PROTOCOL_PACK_LENGTH_020A 12
#define PROTOCOL_PACK_LENGTH_F100 PROTOCOL_PACK_0301_HEADER + 2
#define PROTOCOL_PACK_LENGTH_F101 PROTOCOL_PACK_0301_HEADER + 15
#define PROTOCOL_PACK_LENGTH_F102 PROTOCOL_PACK_0301_HEADER + 30
#define PROTOCOL_PACK_LENGTH_F103 PROTOCOL_PACK_0301_HEADER + 75
#define PROTOCOL_PACK_LENGTH_F104 PROTOCOL_PACK_0301_HEADER + 105
#define PROTOCOL_PACK_LENGTH_F110 PROTOCOL_PACK_0301_HEADER + 45

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
            uint8_t data[PROTOCOL_PACK_LENGTH_0201];
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
            uint8_t data[PROTOCOL_PACK_LENGTH_0202];
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
            uint8_t data[PROTOCOL_PACK_LENGTH_0205];
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
            uint8_t data[PROTOCOL_PACK_LENGTH_0206];
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
            uint8_t data[PROTOCOL_PACK_LENGTH_0207];
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
            uint8_t data[PROTOCOL_PACK_LENGTH_F100];
        };
    };
} client_custom_graphic_delete_t;

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
    union {
        struct {
            uint16_t              data_cmd_id;
            uint16_t              send_id;
            uint16_t              receiver_id;
            graphic_data_struct_t grapic_data_struct;
        };
        struct {
            uint8_t data[PROTOCOL_PACK_LENGTH_F101];
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
            uint8_t data[PROTOCOL_PACK_LENGTH_F102];
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
            uint8_t data[PROTOCOL_PACK_LENGTH_F103];
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
            uint8_t data[PROTOCOL_PACK_LENGTH_F104];
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
            uint8_t               data[30];
        };
        struct {
            uint8_t data[PROTOCOL_PACK_LENGTH_F110];
        };
    };
} client_custom_character_t;

// clang-format off

/**
 * @brief   协议长度与ID列表
 * @note    PROTOCOL_X_LENGTH_FUNCTION
 *          与PROTOCOL_X_ID_ARRAY
 *          与PROTOCOL_X_t
 *          必须一一对应
 */

#define PROTOCOL_JUDGE_LENGTH_FUNCTION(f)     \
        PROTOCOL_PACK_LENGTH_0201             \
      f PROTOCOL_PACK_LENGTH_0202             \
      f PROTOCOL_PACK_LENGTH_0205             \
      f PROTOCOL_PACK_LENGTH_0206             \
      f PROTOCOL_PACK_LENGTH_0207             \
      f PROTOCOL_PACK_LENGTH_F100             \
      f PROTOCOL_PACK_LENGTH_F101             \
      f PROTOCOL_PACK_LENGTH_F102             \
      f PROTOCOL_PACK_LENGTH_F103             \
      f PROTOCOL_PACK_LENGTH_F104             \
      f PROTOCOL_PACK_LENGTH_F110

#define PROTOCOL_JUDGE_ID_ARRAY 0x0201,0x202,0x205,0x206,0x207,0xF100,0xF101,0xF102,0xF103,0xF104,0xF110
#define PROTOCOL_JUDGE_LENGTH   PROTOCOL_JUDGE_LENGTH_FUNCTION(PLUS)

// clang-format on

/**
 * @brief   协议调用接口
 */

typedef struct {
    union {
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
        };
        struct {
            uint8_t data[PROTOCOL_JUDGE_LENGTH];
        };
    };
} PROTOCOL_JUDGE_t;
