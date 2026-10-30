#include "handle.h"

static const uint16_t crc16_ccitt_table[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393,
    0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af,
    0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5, 0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb,
    0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 0xce4c, 0xdfc5, 0xed5e, 0xfcd7,
    0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb,
    0xaa72, 0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e,
    0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c,
    0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff, 0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 0x2942, 0x38cb, 0x0a50,
    0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e,
    0x5cf5, 0x4d7c, 0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d,
    0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
    0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9, 0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9,
    0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

/*
** Descriptions: CRC16 checksum function
** Input: Data to check,channel length, initialized checksum
** Output: CRC checksum
*/
unsigned short Get_CRC16_CCITT(unsigned char *pchMessage, unsigned int dwLength) {
    unsigned short wCRC = 0x0000;
    unsigned char  chData;
    if (pchMessage == 0) {
        return 0xFFFF;
    }
    while (dwLength--) {
        chData = *pchMessage++;
        (wCRC) = ((unsigned short) (wCRC) >> 8) ^ crc16_ccitt_table[((unsigned short) (wCRC) ^ (unsigned short) (chData)) & 0x00ff];
    }
    return wCRC;
}

/**
 * @brief 将浮点物理力矩 (N·m) 转换为电机协议所需的 int16_t 整数
 * @param motor 宇树电机结构体
 * @param tau_ff 前馈力矩 (单位: N·m, 范围 -127.99 ~ +127.99)
 * @return int16_t 写入串口发送帧的数据
 */
int16_t Torque_To_Raw(Unitree_Motor_Type *motor,float tau_ff)
{
    // 1. 安全限幅：防止超出 2 字节 signed short 范围引起数据溢出/翻转
    if (tau_ff > 127.99f)  tau_ff = 127.99f;
    if (tau_ff < -127.99f) tau_ff = -127.99f;

    // 2. 乘 256 并强制类型转换为 signed short (int16_t)
    return (int16_t)(tau_ff * 256.0f * motor->direction / motor->reduction_rate);
}

/**
 * @brief 期望电机速度转换设置值
 * @param motor 宇树电机结构体
 * @param w_des 期望电机速度(rad/s)
 * @return int16_t 写入串口发送帧的数据
 */
int16_t Speed_To_Raw(Unitree_Motor_Type *motor,float w_des){
    if(w_des>804.0f)    w_des=804.0f;
    if(w_des<-804.0f)   w_des=-804.0f;
    return (int16_t)(w_des * 256.0f / 6.28318f * motor->direction * motor->reduction_rate);
}

/**
 * @brief 期望电机位置转换设置值
 * @param motor 宇树电机结构体
 * @param pos_des 期望电机位置(rad)
 * @return int32_t 写入串口发送帧的数据
 */
int32_t Pos_To_Raw(Unitree_Motor_Type *motor,float pos_des){
    return (int32_t)((pos_des * motor->direction * motor->reduction_rate + motor->pos_offset)/ 6.28318f * 32768.0f);
}

/**
 * @brief 关节刚度系数 Kp转换Raw值
 * @param Kp 期待的Kp
 * @return uint16_t 写入串口发送帧的数据
 */
uint16_t Kp_To_Raw(float Kp){
    if(Kp<0.0f)    Kp=0.0f;
    if(Kp>25.599f)  Kp=25.599f;
    return (uint16_t)(Kp*1280.0f);
}

/**
 * @brief 关节阻尼系数 Kd转换Raw值
 * @param Kd 期待的Kd
 * @return uint16_t 写入串口发送帧的数据
 */
uint16_t Kd_To_Raw(float Kd){
    if(Kd<0.0f)    Kd=0.0f;
    if(Kd>25.599f)  Kd=25.599f;
    return (uint16_t)(Kd*1280.0f);
}

/**
 * @brief 实际关节输出位置 (多圈累加)  RAW转连续角(rad)
 * @param motor 电机结构体指针
 * @param raw_pos 原始数据
 * @return float 连续角(rad)
 */
float Raw_To_Angle(Unitree_Motor_Type *motor,int32_t raw_pos){
    // 1. 原始值按满量程 32768 归一化, 乘 2π (6.28318) 得到电机轴原始角度 real_pos (rad)
    float real_pos=(float)(raw_pos / 32768.0f * 6.28318f);
    // 2. 减去零位偏置 -> 乘以方向极性 -> 除以减速比, 得到负载侧实际输出角度
    return (float)((((real_pos)*motor->direction) / motor->reduction_rate)-motor->pos_offset);
}

/**
 * @brief 实际关节输出速度  转速(rad/s)
 * @param motor 电机结构体指针
 * @param raw_speed 原始数据
 * @return float 转速(rad/s)
 */
float Raw_To_Speed(Unitree_Motor_Type *motor,int16_t raw_speed){
    float real_speed=(float)(raw_speed / 256.0f * 6.28318f / motor->reduction_rate);
    return (float)(real_speed*motor->direction);
}

/**
 * @brief 实际关节输出转矩  转矩(N·m)
 * @param motor 电机结构体指针
 * @param raw_torque 原始数据
 * @return 转矩(N·m)
 */
float Raw_To_Torque(Unitree_Motor_Type *motor,int16_t raw_torque){
    float real_torque=(float)(raw_torque / 256.0f * motor->reduction_rate);
    return (float)(real_torque*motor->direction);
}

/**
 * @brief  初始化单个宇树电机
 * @param  motor: 电机结构体指针
 * @param  id: 电机硬件ID (0~14)
 * @param  reduction_rate: 外部减速比 (若电机输出轴直接接连杆则填 1.0f)
 * @param  dir: 旋转方向 (1 为正转, -1 为反转)
 * @param  zero_offset: 机械臂零位校准偏置 (rad)
 */
void Unitree_Motor_Init(Unitree_Motor_Type *motor, 
                        uint8_t id, 
                        float reduction_rate, 
                        int8_t dir, 
                        float zero_offset){
    
    //1.先进行默认配置
    motor->id=id;
    motor->reduction_rate=reduction_rate;
    motor->direction=dir;
    motor->pos_offset=zero_offset;
    motor->status=0;
    
    // 2. 初始安全控制量（上电默认进入阻尼模式，防止突然暴冲甩臂）
    motor->cmd.mode          = 0;              // 0: 阻尼模式 / 停止
    motor->cmd.target_angle  = 0.0f;           // 期望角度
    motor->cmd.target_speed  = 0.0f;           // 期望速度
    motor->cmd.target_torque = 0.0f;           // 前馈力矩
    motor->cmd.kp            = 0.0f;           // 上电初始刚度设为 0
    motor->cmd.kd            = 1.0f;           // 给少量阻尼 Kd，起到缓冲制动保护作用

    // 3. 反馈状态初始化
    motor->state.angle       = 0.0f;
    motor->state.speed       = 0.0f;
    motor->state.torque      = 0.0f;
    motor->state.temperature = 0;
    motor->state.online      = 0;              // 刚上电标记为离线，收到第一帧心跳后置 1
    motor->state.error_code  = 0;
    motor->state.updated_at  = 0;
}

/**
 * @brief Unitree通讯桥初始化
 * @param bridge   通讯桥变量
 * @param usartx    Usartx
 * @param deviceID  Usartx对应的x的值
 */
void Unitree_Bridge_Init(Unitree_Bridge_Type *bridge,
                         USART_TypeDef *usartx,uint32_t deviceID){
    BSP_DMA_Init(USARTx_Rx, bridge->rx_buf, sizeof(Unitree_RecvFrame_t));
    BSP_DMA_Init(USARTx_Tx, bridge->tx_buf, sizeof(Unitree_SendFrame_t));
    bridge->usart=usartx;
    bridge->deviceID=deviceID;
    bridge->motor_count=0;
    bridge->polling_index=0;
    bridge->last_send_time=0;
    bridge->IsBusy=0;
}

/**
 * @brief Unitree电机绑定Unitree桥
 * @param bridge    通讯桥变量
 * @param motor     Unitree电机变量
 */
void Unitree_Bridge_Bind(Unitree_Bridge_Type *bridge,Unitree_Motor_Type *motor){
    bridge->motors[motor->id]=motor;
    bridge->motor_count++;
}

/**
 * @brief 解析宇树电机反馈帧 (小端字节序)
 * @param bridge 通讯桥变量 (用于更新电机在线状态)
 * @param frame  解析结果输出结构体
 * @param data   原始接收字节流 (长度 >= sizeof(Unitree_RecvFrame_t))
 * @note  调用前应先校验 CRC
 */
void Unitree_Unpack(Unitree_Bridge_Type *bridge, Unitree_RecvFrame_t *frame, uint8_t *data){
    
    memcpy(frame,data,sizeof(Unitree_RecvFrame_t));

    //更新对应电机的状态 (心跳 + 反馈数据)
    if (bridge != NULL) {
        uint8_t id = frame->id & 0x0F;
        // 先检查 ID 是否在已绑定范围内, 防止数组越界
        if (id < bridge->motor_count) {
            Unitree_Motor_Type *motor = bridge->motors[id];
            if (motor != NULL && motor->id == frame->id) {
                //置允许发送标志位
                motor->status            = 0;
                
                //在线心跳
                if(frame->error_code==0){
                    motor->state.online = 1;
                }else{
                    //电机返回错误码则给online置故障标志位
                    motor->state.online = 2;
                }
                motor->state.error_code = frame->error_code;
                motor->state.updated_at = xTaskGetTickCount();

                //反馈数据 (考虑方向极性, 角度叠加零偏)
                motor->state.angle       = Raw_To_Angle(motor,frame->pos);
                motor->state.speed       = Raw_To_Speed(motor,frame->speed);
                motor->state.torque      = Raw_To_Torque(motor,frame->torque);
                motor->state.temperature = frame->temperature;
            }
        }
    }
}

/**
 * @brief 宇树电机接收处理 (IDLE 中断中调用)
 * @param bridge   通讯桥变量
 * @note  流程: 清 IDLE 标志 -> 停 DMA -> 帧头校验 -> CRC 校验 -> 解包 -> 重启 DMA
 */
void Unitree_Receive(Unitree_Bridge_Type *bridge){
    // 接收到数据，将总线繁忙标志位置空闲
    bridge->IsBusy=0;

    uint16_t tmp, len;
    uint32_t deviceID;
    deviceID=bridge->deviceID;
    // 1. 清除 IDLE 标志: 必须先读 SR 再读 DR (顺序不能反, 否则标志清不掉导致中断风暴)
    tmp = USARTx->SR;
    tmp = USARTx->DR;

    // 2. 停止 DMA, 防止新数据覆盖当前帧
    DMA_Disable(USARTx_Rx);

    // 3. 计算实际接收长度 (缓冲区大小 - DMA 剩余计数)
    len = UNITREE_BUFFER_SIZE - DMA_Get_Data_Counter(USARTx_Rx);

    // 4. 帧头校验 (快速过滤垃圾数据, 帧头不对直接丢弃, 不算 CRC)
    if (len >= sizeof(Unitree_RecvFrame_t) &&
        bridge->rx_buf[0] == 0xFD && bridge->rx_buf[1] == 0xEE) {
        uint16_t recv_crc = (uint16_t)bridge->rx_buf[14] | ((uint16_t)bridge->rx_buf[15] << 8);
        // 5. CRC16 校验 (确认数据完整无误)
        if (Get_CRC16_CCITT((uint8_t *)bridge->rx_buf, offsetof(Unitree_RecvFrame_t,crc16))==recv_crc) {
            // 6. 校验通过, 解包并更新电机状态
            Unitree_RecvFrame_t frame;
            Unitree_Unpack(bridge, &frame, bridge->rx_buf);
        }
    }

    // 7. 重新使能 DMA, 接收下一帧
    DMA_Enable(USARTx_Rx, UNITREE_BUFFER_SIZE);
}

/**
 * @brief 宇树总线看门狗：防止丢包导致总线死锁
 * @param bridge 通讯桥变量
 */
void Unitree_Bus_Watchdog(Unitree_Bridge_Type *bridge) {
    if (bridge->IsBusy == 0) {
        return; // 总线本来就是空闲的，无需处理
    }

    // 检查从发送到现在的等待时间是否超过阈值
    if ((xTaskGetTickCount() - bridge->last_send_time) > UNITREE_TIMEOUT_TICKS) {
        // 1. 获取当前等待超时的电机
        uint8_t timeout_id = bridge->polling_index;
        if (timeout_id < bridge->motor_count && bridge->motors[timeout_id] != NULL) {
            Unitree_Motor_Type *motor = bridge->motors[timeout_id];
            
            // 2. 解除该电机的等待标志（允许下次继续发给它或跳过它）
            motor->status = 0;

            // 3. 统计连续丢帧或更新离线状态
            // 若距离最后一次收到有效帧超过 50ms，则判定电机离线
            if (xTaskGetTickCount() - motor->state.updated_at > pdMS_TO_TICKS(50)) {
                //给故障标志位
                motor->state.online = 2;
            }
        }

        // 4. 强制释放总线，避免死锁
        bridge->IsBusy = 0;

        // 5. 复位 DMA 接收端，清空可能被噪声污染的残缺数据
        uint32_t deviceID = bridge->deviceID;
        DMA_Disable(USARTx_Rx);
        DMA_Enable(USARTx_Rx, UNITREE_BUFFER_SIZE);
    }
}

/**
 * @brief 生成电机17byte发送帧
 * @param *frame 帧指针
 * @param *motor 电机指针
 */
void Unitree_Generate_SendFrame(Unitree_SendFrame_t *frame,Unitree_Motor_Type *motor){
    memset(frame, 0, sizeof(Unitree_SendFrame_t));
    frame->head[0]=0xFE;
    frame->head[1]=0xEE;
    frame->id=(motor->id)&0x0F;
    frame->mode=motor->cmd.mode;
    frame->target_pos=Pos_To_Raw(motor,motor->cmd.target_angle);
    frame->target_speed=Speed_To_Raw(motor,motor->cmd.target_speed);
    frame->target_torque=Torque_To_Raw(motor,motor->cmd.target_torque);
    frame->kp=Kp_To_Raw(motor->cmd.kp);
    frame->kd=Kd_To_Raw(motor->cmd.kd);
    frame->crc16=Get_CRC16_CCITT((uint8_t *)frame,offsetof(Unitree_SendFrame_t, crc16));
}

/**
 * @brief 单个电机发送函数
 * @param *bridge Unitree电机桥指针
 * @param motorId 电机ID
 */
void Unitree_Motor_Send(Unitree_Bridge_Type *bridge,uint8_t motorId){
    uint32_t deviceID=bridge->deviceID;
    Unitree_Motor_Type *motor=bridge->motors[motorId];
    Unitree_SendFrame_t frame;
    if(motor->status==1 || bridge->IsBusy==1 || motor->state.online==2){
        //电机online==2则为故障，也直接不推进轮询
        //目前没有电机掉线机械臂还要继续运行的需求，特此备注
        return;
    }
    bridge->polling_index=motorId;
    Unitree_Generate_SendFrame(&frame,motor);
    memcpy(bridge->tx_buf,&frame,sizeof(Unitree_SendFrame_t));
    motor->status=1;
    bridge->IsBusy=1;
    bridge->last_send_time = xTaskGetTickCount();
    DMA_Disable(USARTx_Tx);
    DMA_Enable(USARTx_Tx,sizeof(Unitree_SendFrame_t));
}

/**
 * @brief 电机桥循环发送
 * @param *bridge Unitree电机桥指针
 */
void Unitree_Circular_Send(Unitree_Bridge_Type *bridge){
    Unitree_Bus_Watchdog(bridge);
    if (bridge->IsBusy == 1) {
        return; // 总线正忙，等待当前电机完成收发，不推进索引
    }
    uint8_t now_id=bridge->polling_index;
    now_id++;
    if(now_id>=bridge->motor_count){
        now_id=0;
    }
    Unitree_Motor_Send(bridge,now_id);
}

/**
 * @brief  宇树8010电机安全单帧测试函数 (先发一帧拿反馈，超低风险)
 * @param  bridge: 通讯桥指针 (如 &Unitree_Bridge)
 * @param  motor_id: 测试电机的ID (0~14, 如 0 号电机)
 * @param  timeout_ms: 等待反馈超时时间 (推荐 10~50 ms)
 * @return uint8_t: 1 表示收到有效反馈且电机在线, 0 表示超时未收到反馈
 */
uint8_t Unitree_Motor_Safety_Test(Unitree_Bridge_Type *bridge, uint8_t motor_id, uint32_t timeout_ms)
{
    if (bridge == NULL || motor_id >= UNITREE_MAX_MOTORS_PER_BUS) {
        return 0;
    }
    Unitree_Motor_Type *motor = bridge->motors[motor_id];
    if (motor == NULL) {
        return 0;
    }

    /* 1. 设置极度安全的控制参数 (阻尼/停止模式，零力矩，零刚度) */
    motor->cmd.mode          = 0;     // 0: 阻尼模式 / 停止 (不输出驱动力矩，安全等级最高)
    motor->cmd.target_angle  = 0.0f;  // 期望角度 0
    motor->cmd.target_speed  = 0.0f;  // 期望速度 0
    motor->cmd.target_torque = 0.0f;  // 前馈力矩 0 N·m
    motor->cmd.kp            = 0.0f;  // 刚度 0 (完全不产生位置恢复力)
    motor->cmd.kd            = 0.0f;  // 阻尼 0 (纯自由读取状态，不阻碍转动)

    /* 2. 状态重置与准备 */
    //motor->state.online = 0;          // 清除在线标志，用来判断本次是否成功拿到反馈
    //motor->status       = 0;          // 允许发送
    //bridge->IsBusy      = 0;          // 释放总线

    /* 3. 发送这一帧指令 */
    // Unitree_Motor_Send(bridge, motor_id);

    // /* 4. 等待中断接收反馈 (阻塞等待接收中断更新 online 标志) */
    // uint32_t wait_ticks = 0;
    // while (motor->state.online == 0 && wait_ticks < (timeout_ms * 1000)) {
    //     // 微秒级/简易空延时(约1us)，若在FreeRTOS任务中可改为 vTaskDelay(pdMS_TO_TICKS(1))
    //     for (volatile int i = 0; i < 30; i++); 
    //     wait_ticks++;
    // }

    // /* 5. 判断结果 */
    // if (motor->state.online == 1) {
    //     // 已成功收到电机反馈帧！
    //     // 此时可以在调试断点/观察窗口查看:
    //     // motor->state.angle       (当前实际角度 rad)
    //     // motor->state.speed       (当前实际速度 rad/s)
    //     // motor->state.torque      (当前输出力矩 N*m)
    //     // motor->state.temperature (当前电机温度 ℃)
    //     return 1;
    // } else {
    //     // 超时未收到反馈，检查 RS485 收发器使能、接线(A/B)、波特率(4Mbps)或电机ID是否匹配
    //     return 0;
    // }
}

/**
 * @brief 宇树电机根据参数前往目标角度
 * @param *self 机械臂关节指针
 * @param *joint_command 电机指令结构体 
 * @return void
 */
void Unitree_Set_Target(Arm_Joint_Type *self, Joint_Command_t *joint_command){
    Unitree_Motor_Type* motor=(Unitree_Motor_Type*)(self->motor_handle);
    motor->cmd.mode=1;
    motor->cmd.target_angle=joint_command->pos_des;
    motor->cmd.target_speed=joint_command->vel_des;
    motor->cmd.target_torque=joint_command->tau_ff;
    motor->cmd.kp=joint_command->kp;
    motor->cmd.kd=joint_command->kd;
}

/**
 * @brief 宇树电机获取电机角度
 * @param *self 机械臂关节指针
 * @return float:返回电机当前角度(rad)
 */
float Unitree_Get_Angle(Arm_Joint_Type *self){
    Unitree_Motor_Type* motor=(Unitree_Motor_Type*)(self->motor_handle);
    return (float)motor->state.angle;
}

/**
 * @brief 宇树电机获取电机速度
 * @param *self 机械臂关节指针
 * @return float:返回电机当前速度(rad/s)
 */
float Unitree_Get_Speed(Arm_Joint_Type *self){
    Unitree_Motor_Type* motor=(Unitree_Motor_Type*)(self->motor_handle);
    return (float)motor->state.speed;
}

/**
 * @brief 宇树电机获取在线状态
 * @param *self 机械臂关节指针
 * @return uint8_t:返回电机在线状态(1:在线 或0:不在线)
 */
uint8_t Unitree_Is_Online(Arm_Joint_Type *self){
    Unitree_Motor_Type* motor=(Unitree_Motor_Type*)(self->motor_handle);
    if(motor->state.online==1){
        return 1;
    }else{
        return 0;
    }
}