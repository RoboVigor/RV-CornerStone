#ifndef __DRIVER_PROTOCOL_H
#define __DRIVER_PROTOCOL_H

#include "stm32f4xx.h"

#define Protocol_Buffer_Length 128

#define Protocol_Data_Id_Client 0xD180
#define Protocol_Data_Id_Robot                                                                                                                                 \
    0x0200 : case 0x0201 : case 0x0202 : case 0x0203 : case 0x0204 : case 0x0205 : case 0x0206 : case 0x0207 : case 0x0208 : case 0x0209 : case 0x020a         \
        : case 0x020b : case 0x020c : case 0x020d : case 0x020e : case 0x020f : case 0x0210 : case 0x0211 : case 0x0212 : case 0x0213 : case 0x0214            \
        : case 0x0215 : case 0x0216 : case 0x0217 : case 0x0218 : case 0x0219 : case 0x021a : case 0x021b : case 0x021c : case 0x021d : case 0x021e            \
        : case 0x021f : case 0x0220 : case 0x0221 : case 0x0222 : case 0x0223 : case 0x0224 : case 0x0225 : case 0x0226 : case 0x0227 : case 0x0228            \
        : case 0x0229 : case 0x022a : case 0x022b : case 0x022c : case 0x022d : case 0x022e : case 0x022f : case 0x0230 : case 0x0231 : case 0x0232            \
        : case 0x0233 : case 0x0234 : case 0x0235 : case 0x0236 : case 0x0237 : case 0x0238 : case 0x0239 : case 0x023a : case 0x023b : case 0x023c            \
        : case 0x023d : case 0x023e : case 0x023f : case 0x0240 : case 0x0241 : case 0x0242 : case 0x0243 : case 0x0244 : case 0x0245 : case 0x0246            \
        : case 0x0247 : case 0x0248 : case 0x0249 : case 0x024a : case 0x024b : case 0x024c : case 0x024d : case 0x024e : case 0x024f : case 0x0250            \
        : case 0x0251 : case 0x0252 : case 0x0253 : case 0x0254 : case 0x0255 : case 0x0256 : case 0x0257 : case 0x0258 : case 0x0259 : case 0x025a            \
        : case 0x025b : case 0x025c : case 0x025d : case 0x025e : case 0x025f : case 0x0260 : case 0x0261 : case 0x0262 : case 0x0263 : case 0x0264            \
        : case 0x0265 : case 0x0266 : case 0x0267 : case 0x0268 : case 0x0269 : case 0x026a : case 0x026b : case 0x026c : case 0x026d : case 0x026e            \
        : case 0x026f : case 0x0270 : case 0x0271 : case 0x0272 : case 0x0273 : case 0x0274 : case 0x0275 : case 0x0276 : case 0x0277 : case 0x0278            \
        : case 0x0279 : case 0x027a : case 0x027b : case 0x027c : case 0x027d : case 0x027e : case 0x027f : case 0x0280 : case 0x0281 : case 0x0282            \
        : case 0x0283 : case 0x0284 : case 0x0285 : case 0x0286 : case 0x0287 : case 0x0288 : case 0x0289 : case 0x028a : case 0x028b : case 0x028c            \
        : case 0x028d : case 0x028e : case 0x028f : case 0x0290 : case 0x0291 : case 0x0292 : case 0x0293 : case 0x0294 : case 0x0295 : case 0x0296            \
        : case 0x0297 : case 0x0298 : case 0x0299 : case 0x029a : case 0x029b : case 0x029c : case 0x029d : case 0x029e : case 0x029f : case 0x02a0            \
        : case 0x02a1 : case 0x02a2 : case 0x02a3 : case 0x02a4 : case 0x02a5 : case 0x02a6 : case 0x02a7 : case 0x02a8 : case 0x02a9 : case 0x02aa            \
        : case 0x02ab : case 0x02ac : case 0x02ad : case 0x02ae : case 0x02af : case 0x02b0 : case 0x02b1 : case 0x02b2 : case 0x02b3 : case 0x02b4            \
        : case 0x02b5 : case 0x02b6 : case 0x02b7 : case 0x02b8 : case 0x02b9 : case 0x02ba : case 0x02bb : case 0x02bc : case 0x02bd : case 0x02be            \
        : case 0x02bf : case 0x02c0 : case 0x02c1 : case 0x02c2 : case 0x02c3 : case 0x02c4 : case 0x02c5 : case 0x02c6 : case 0x02c7 : case 0x02c8            \
        : case 0x02c9 : case 0x02ca : case 0x02cb : case 0x02cc : case 0x02cd : case 0x02ce : case 0x02cf : case 0x02d0 : case 0x02d1 : case 0x02d2            \
        : case 0x02d3 : case 0x02d4 : case 0x02d5 : case 0x02d6 : case 0x02d7 : case 0x02d8 : case 0x02d9 : case 0x02da : case 0x02db : case 0x02dc            \
        : case 0x02dd : case 0x02de : case 0x02df : case 0x02e0 : case 0x02e1 : case 0x02e2 : case 0x02e3 : case 0x02e4 : case 0x02e5 : case 0x02e6            \
        : case 0x02e7 : case 0x02e8 : case 0x02e9 : case 0x02ea : case 0x02eb : case 0x02ec : case 0x02ed : case 0x02ee : case 0x02ef : case 0x02f0            \
        : case 0x02f1 : case 0x02f2 : case 0x02f3 : case 0x02f4 : case 0x02f5 : case 0x02f6 : case 0x02f7 : case 0x02f8 : case 0x02f9 : case 0x02fa            \
        : case 0x02fb : case 0x02fc : case 0x02fd : case 0x02fe : case 0x02ff

#define Protocol_Pack_Length_0001 3
#define Protocol_Pack_Length_0002 1
#define Protocol_Pack_Length_0003 2
#define Protocol_Pack_Length_0101 4
#define Protocol_Pack_Length_0102 3
#define Protocol_Pack_Length_0103 2
#define Protocol_Pack_Length_0201 15
#define Protocol_Pack_Length_0202 14
#define Protocol_Pack_Length_0203 16
#define Protocol_Pack_Length_0204 1
#define Protocol_Pack_Length_0205 3
#define Protocol_Pack_Length_0206 1
#define Protocol_Pack_Length_0207 6
#define Protocol_Pack_Length_0301_Header 6
#define Protocol_Pack_Length_0301_Client 13
#define Protocol_Pack_Length_0301_Robot 112
#define Protocol_Pack_Length_0401 9

#define PROTOCOL_HEADER 0xA5
#define PROTOCOL_HEADER_SIZE 5
#define PROTOCOL_CMD_SIZE 2
#define PROTOCOL_CRC16_SIZE 2
#define PROTOCOL_HEADER_CRC_LEN (PROTOCOL_HEADER_SIZE + PROTOCOL_CRC16_SIZE)
#define PROTOCOL_HEADER_CRC_CMDID_LEN (PROTOCOL_HEADER_SIZE + PROTOCOL_CRC16_SIZE + PROTOCOL_CMD_SIZE)
#define PROTOCOL_HEADER_CMDID_LEN (PROTOCOL_HEADER_SIZE + PROTOCOL_CMD_SIZE)

typedef union {
    uint8_t  U8[4];
    uint16_t U16[2];
    uint32_t U32;
    float    F;
    int      I;
} format_trans_t;

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
            uint8_t  mains_power_gimbal_output : 1;
            uint8_t  mains_power_chassis_output : 1;
            uint8_t  mains_power_shooter_output : 1;
        };
        struct {
            uint8_t data[Protocol_Pack_Length_0201];
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
        };
        struct {
            uint8_t data[Protocol_Pack_Length_0202];
        };
    };
} ext_power_heat_data_t;

typedef struct {
    union {
        struct {
            uint8_t energy_point;
            uint8_t attack_time;
        };
        struct {
            uint8_t data[Protocol_Pack_Length_0205];
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
            uint8_t data[Protocol_Pack_Length_0206];
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
            uint8_t data[Protocol_Pack_Length_0207];
        };
    };
} ext_shoot_data_t;

typedef struct {
    union {
        struct {
            float   yaw_angle_diff;
            float   pitch_angle_diff;
            uint8_t biu_biu_state;
        };
        struct {
            uint8_t data[Protocol_Pack_Length_0401];
        };
    };
} ext_gimal_aim_data_t;

typedef struct {
    union {
        struct {
            uint16_t data_cmd_id;
            uint16_t send_id;
            uint16_t receiver_id;
            float    data1;
            float    data2;
            float    data3;
            uint8_t  masks;
        };
        struct {
            uint8_t data[Protocol_Pack_Length_0301_Header + Protocol_Pack_Length_0301_Client];
        };
    };
} client_custom_data_t;

typedef struct {
    union {
        struct {
            uint16_t       data_cmd_id;
            uint16_t       send_id;
            uint16_t       receiver_id;
            format_trans_t transformer[Protocol_Pack_Length_0301_Robot / sizeof(float)];
        };
        struct {
            uint8_t data[Protocol_Pack_Length_0301_Header + Protocol_Pack_Length_0301_Robot];
        };
    };
} robot_interactive_data_t;

typedef struct {
    union {
        struct {
            uint16_t data1;
        };
        struct {
            uint8_t data[2];
        };
    };
} vision_interactive_data_t;

typedef struct {
    uint8_t  sof;
    uint16_t data_length;
    uint8_t  seq;
    uint8_t  crc8;
} frame_header_t;

typedef enum {
    STEP_HEADER_SOF  = 0,
    STEP_LENGTH_LOW  = 1,
    STEP_LENGTH_HIGH = 2,
    STEP_FRAME_SEQ   = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct {
    uint8_t                   buf[Protocol_Buffer_Length];
    uint8_t                   packet[Protocol_Buffer_Length];
    uint8_t                   interact[Protocol_Buffer_Length];
    unpack_step_e             step;
    uint16_t                  index;
    uint16_t                  dataLength;
    uint16_t                  seq;
    uint16_t                  id;
    uint64_t                  lost;
    uint64_t                  received;
    ext_game_robot_state_t    robotState;
    ext_power_heat_data_t     powerHeatData;
    aerial_robot_energy_t     aerialRobotEnergy;
    ext_robot_hurt_t          robotHurt;
    ext_shoot_data_t          shootData;
    ext_gimal_aim_data_t      gimbalAimData;
    client_custom_data_t      clientCustomData;
    vision_interactive_data_t visionInteractiveData;
    robot_interactive_data_t  robotInteractiveData[8];
} Protocol_Type;

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
unsigned int  Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void          Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t      Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t      Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void          Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

void Protocol_Init(Protocol_Type *Protocol);
void Protocol_Update(Protocol_Type *Protocol);
void Protocol_Unpack(Protocol_Type *Protocol, uint8_t byte);
void Protocol_Load(Protocol_Type *Protocol);
void Protocol_Pack(Protocol_Type *Protocol, int length, uint16_t id);

#endif
