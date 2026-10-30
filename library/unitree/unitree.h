#ifndef __UNITREE_H
#define __UNITREE_H

#include "Driver_MecArm.h"

#define UNITREE_MAX_MOTORS_PER_BUS  2
#define UNITREE_BUFFER_SIZE         128
#define UNITREE_TIMEOUT_TICKS  pdMS_TO_TICKS(2) // 超时阈值：2ms

#pragma pack(1)

/**
 * @brief 宇树电机下发控制数据包
 */
typedef struct {
    uint8_t  head[2];             // 帧头，例如 0xFE, 0xEE
    uint8_t  id             :4;   // 电机硬件 ID (0~14)
    uint8_t  mode           :3;   // 工作模式 (0: 阻尼/停止, 1: FOC闭环运控, 2: 校准)
    uint8_t                 :1;   // 占位 
    // 宇树的核心 5 参数阻抗控制模型:
    // T_output = tau_ff + Kp * (pos_des - pos_real) + Kd * (vel_des - vel_real)
    int16_t  target_torque;       // 前馈力矩 tau_ff (N·m)
    int16_t  target_speed;        // 期望角速度 vel_des (rad/s)
    int32_t  target_pos;          // 期望角度 pos_des (rad)
    uint16_t kp;                  // 关节刚度系数 Kp
    uint16_t kd;                  // 关节阻尼系数 Kd
    
    uint16_t crc16;               // CRC16 校验码
} Unitree_SendFrame_t;

/**
 * @brief 宇树电机反馈数据包
 */
typedef struct {
    uint8_t  head[2];               // 帧头
    uint8_t  id             :4;     // 电机 ID
    uint8_t  mode           :3;     // 当前运行模式
    uint8_t                 :1;     // 占位

    int16_t  torque;                // 当前实际输出力矩 (N·m)
    int16_t  speed;                 // 当前输出轴角速度 (rad/s)
    int32_t  pos;                   // 当前输出轴物理位置 (rad)
    
    int8_t   temperature;           // 电机温度 (℃)
    uint16_t error_code     :3;     // 故障状态字 (0为正常)
    uint16_t force          :12;    // 足端力
    uint16_t                 :1;     // 占位
    
    uint16_t crc16;                 // CRC16 校验码
} Unitree_RecvFrame_t;

#pragma pack()

typedef struct {
    // 1. 静态参数与配置
    uint8_t  id;                // 电机 CAN/RS485 ID
    int8_t   direction;         // 旋转方向极性 (1: 正向, -1: 反向)
    uint8_t  status;            // 电机目前是接收还是可发送状态(1:接收，0:发送)
    float    reduction_rate;    // 减速比 (如 6.33, 9.0)
    float    pos_offset;        // 关节零点偏移量 (rad)
    
    // 2. 控制指令 (上层算法写入)
    struct {
        uint8_t mode;           // 模式控制
        float   target_angle;   // 关节期望角度 (rad)
        float   target_speed;   // 期望角速度 (rad/s)
        float   target_torque;  // 前馈力矩 (N·m)
        float   kp;             // 刚度增益
        float   kd;             // 阻尼增益
    } cmd;

    // 3. 状态反馈 (解包后更新)
    struct {
        float      angle;       // 经过零偏计算后的关节实际角度 (rad)
        float      speed;       // 关节实际角速度 (rad/s)
        float      torque;      // 关节实际输出力矩 (N·m)
        int8_t     temperature; // 温度(摄氏度)
        uint8_t    online;      // 在线标志位 (1: 在线, 0: 离线，2：故障)
        uint8_t    error_code;  // 错误代码，参见8010电机反馈【电机错误标识】
        TickType_t updated_at;  // 最后收到反馈的心跳时间戳 (用于看门狗)
    } state;

} Unitree_Motor_Type;

typedef struct {
    USART_TypeDef      *usart;                          // 绑定的物理串口 (如 USART6)
    uint32_t            deviceID;                       //USART的ID，方便使用自己维护的usart表
    Unitree_Motor_Type *motors[UNITREE_MAX_MOTORS_PER_BUS]; // 挂在该总线上的电机指针数组
    uint8_t             motor_count;                    // 当前挂载的电机数量
    uint8_t             polling_index;                  // 当前轮询到的电机序号
    
    // DMA 收发缓冲区
    uint8_t             tx_buf[UNITREE_BUFFER_SIZE];
    uint8_t             rx_buf[UNITREE_BUFFER_SIZE];
    
    // 状态控制
    uint8_t             IsBusy;                         // 总线状态（1：繁忙，0：空闲）
    TickType_t          last_send_time;                 // 发送时间戳 (用于超时检测)
} Unitree_Bridge_Type;


unsigned short Get_CRC16_CCITT(unsigned char *pchMessage, unsigned int dwLength);
void Unitree_Motor_Init(Unitree_Motor_Type *motor, 
                        uint8_t id, 
                        float reduction_rate, 
                        int8_t dir, 
                        float zero_offset);
void Unitree_Bridge_Init(Unitree_Bridge_Type *bridge,
                         USART_TypeDef *usartx,uint32_t deviceID);
void Unitree_Bridge_Bind(Unitree_Bridge_Type *bridge,Unitree_Motor_Type *motor);
void Unitree_Receive(Unitree_Bridge_Type *bridge);
void Unitree_Circular_Send(Unitree_Bridge_Type *bridge);
uint8_t Unitree_Motor_Safety_Test(Unitree_Bridge_Type *bridge, uint8_t motor_id, uint32_t timeout_ms);
void Unitree_Set_Target(Arm_Joint_Type *self, Joint_Command_t *joint_command);
float Unitree_Get_Angle(Arm_Joint_Type *self);
float Unitree_Get_Speed(Arm_Joint_Type *self);
uint8_t Unitree_Is_Online(Arm_Joint_Type *self);

#endif
