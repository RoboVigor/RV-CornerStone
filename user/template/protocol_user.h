/**
 * @brief   用户通讯协议
 * @note    id: 0x0501 - 0x05FF 板间通讯
 * @note    id: 0xF201 - 0xF2FF 车间通讯
 * @version 1.0
 */

/**
 * @brief   协议数据段长度
 * @note    单位 bit
 */

#define PROTOCOL_PACK_LENGTH_0501 16
#define PROTOCOL_PACK_LENGTH_0502 16
#define PROTOCOL_PACK_LENGTH_F201 PROTOCOL_PACK_0301_HEADER + 16

/**
 * @brief   协议列表
 */

typedef struct {
    union {
        struct {
            float data1;
            float data2;
            float data3;
            float data4;
        };
        struct {
            uint8_t data[PROTOCOL_PACK_LENGTH_0501];
        };
    };
} board_interactive_data_t;

typedef struct {
    union {
        struct {
            uint16_t data_cmd_id;
            uint16_t send_id;
            uint16_t receiver_id;
            float    data1;
            float    data2;
            float    data3;
            float    data4;
        };
        struct {
            uint8_t data[PROTOCOL_PACK_LENGTH_F201];
        };
    };
} robot_interactive_data_t;

// clang-format off

/**
 * @brief   协议长度与ID列表
 * @note    PROTOCOL_X_LENGTH_FUNCTION
 *          与PROTOCOL_X_ID_ARRAY
 *          与PROTOCOL_X_t
 *          必须一一对应
 */

#define PROTOCOL_USER_LENGTH_FUNCTION(f)     \
        PROTOCOL_PACK_LENGTH_0501             \
      f PROTOCOL_PACK_LENGTH_0502             \
      f PROTOCOL_PACK_LENGTH_F201

#define PROTOCOL_USER_ID_ARRAY 0x0501,0x0502,0xF201
#define PROTOCOL_USER_LENGTH   PROTOCOL_USER_LENGTH_FUNCTION(PLUS)

// clang-format on

/**
 * @brief   协议调用接口
 */

typedef struct {
    union {
        struct {
            board_interactive_data_t boardAlpha;         // 板间通讯
            board_interactive_data_t boardBeta;          // 板间通讯
            robot_interactive_data_t robotCommunication; // 车间通讯
        };
        struct {
            uint8_t data[PROTOCOL_USER_LENGTH];
        };
    };
} PROTOCOL_USER_t;
