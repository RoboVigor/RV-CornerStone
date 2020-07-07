/**
 * @brief   上位机通讯协议
 * @note    id: 0x0401 - 0x04FF
 * @version 1.0
 */

/**
 * @brief   协议数据段长度
 * @note    单位 bit
 */

#define PROTOCOL_PACK_LENGTH_0401 9

/**
 * @brief   协议列表
 */

typedef struct {
    union {
        struct {
            float   yaw_angle_diff;
            float   pitch_angle_diff;
            uint8_t biu_biu_state;
        };
        struct {
            uint8_t data[PROTOCOL_PACK_LENGTH_0401];
        };
    };
} ext_gimbal_aim_data_t;

// clang-format off

/**
 * @brief   协议长度与ID列表
 * @note    PROTOCOL_X_LENGTH_FUNCTION
 *          与PROTOCOL_X_ID_ARRAY
 *          与PROTOCOL_X_t
 *          必须一一对应
 */

#define PROTOCOL_HOST_LENGTH_FUNCTION(f) PROTOCOL_PACK_LENGTH_0401
#define PROTOCOL_HOST_ID_ARRAY 0x0401
#define PROTOCOL_HOST_LENGTH   PROTOCOL_HOST_LENGTH_FUNCTION(PLUS)

// clang-format on

/**
 * @brief   协议调用接口
 */

typedef struct {
    union {
        struct {
            ext_gimbal_aim_data_t autoaimData; // 视觉自瞄数据
        };
        struct {
            uint8_t data[PROTOCOL_HOST_LENGTH];
        };
    };
} PROTOCOL_HOST_t;
