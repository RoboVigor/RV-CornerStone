#include "mpu6500_driver.h"
#include "mpu6500_i2c.h"
#include "mpu6500_IST8310.h"

/*----I2C1----SCL----PB6---*/
/*----I2C1----SDA----PB7---*/

#define IIC_SCL_H() PFout(7) = 1
#define IIC_SCL_L() PFout(7) = 0
#define IIC_SDA_H() PFout(9) = 1
#define IIC_SDA_L() PFout(9) = 0
#define IIC_SDA_Read() PFin(9)

void IIC_Delay(unsigned int t) {
    int i;
    for (i = 0; i < t; i++) {
        int a = 100; // 6
        while (a--)
            ;
    }
}

void IIC_GPIO_Init(void) {
    PFout(6) = 1;
    PFout(8) = 0;
}

void IIC_SDA_Out(void) {
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin   = GPIO_Pin_9;
    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOF, &gpio);
}

void IIC_SDA_In(void) {
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin   = GPIO_Pin_9;
    gpio.GPIO_Mode  = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOF, &gpio);
}

unsigned char IIC_Start(void) {
    IIC_SDA_Out();
    IIC_SDA_H();
    if (!IIC_SDA_Read()) {
        printf("error 1!\r\n");
        return 0xff;
    }
    IIC_SCL_H();
    IIC_Delay(1);
    IIC_SDA_L();
    if (IIC_SDA_Read()) {
        printf("error 2!\r\n");
        return 0xff;
    }

    IIC_Delay(1);
    IIC_SCL_L();
    return 0;
}

void IIC_Stop(void) {
    IIC_SDA_Out();
    IIC_SCL_L();
    IIC_SDA_L();
    IIC_Delay(1);
    IIC_SCL_H();
    IIC_Delay(1);
    IIC_SDA_H();
}

void IIC_Ack(unsigned char re) {
    IIC_SCL_L();
    IIC_SDA_Out();
    IIC_Delay(1);
    if (re)
        IIC_SDA_H();
    else
        IIC_SDA_L();
    IIC_Delay(1);
    IIC_SCL_H();
    IIC_Delay(1);
    IIC_SCL_L();
}

int IIC_WaitAck(void) {
    unsigned short Out_Time = 0; //??2000
    IIC_SDA_In();
    IIC_SDA_H();
    IIC_Delay(1);
    IIC_SCL_H();
    IIC_Delay(1);
    while (IIC_SDA_Read()) {
        Out_Time++;
        if ((Out_Time) > 50) {
            IIC_Stop();

            //      printf("error 2A\r\n");
            return 0xff;
        }
        IIC_Delay(5);
    }
    IIC_SCL_L();
    return 0;
}

void IIC_WriteBit(unsigned char Temp) {
    unsigned char i;
    IIC_SDA_Out();
    IIC_SCL_L();
    for (i = 0; i < 8; i++) {
        if (Temp & 0x80) {
            IIC_SDA_H();
        } else {
            IIC_SDA_L();
        }
        Temp <<= 1;
        IIC_Delay(1);
        IIC_SCL_H();
        IIC_Delay(1);
        IIC_SCL_L();
        IIC_Delay(1);
    }
}

unsigned char IIC_ReadBit(void) {
    unsigned char i, Temp = 0;
    IIC_SDA_In();
    IIC_Delay(1);
    for (i = 0; i < 8; i++) {
        IIC_SCL_L();
        IIC_Delay(2);
        IIC_SCL_H();
        Temp <<= 1;
        if (IIC_SDA_Read()) Temp++;
        IIC_Delay(2);
    }
    IIC_SCL_L();
    return Temp;
}

int IIC_WriteData(unsigned char dev_addr, unsigned char reg_addr, unsigned char data) {
    if (IIC_Start() == 0xff) {
        printf("error 3!");
        return 0xff;
    }

    IIC_WriteBit(dev_addr);
    if (IIC_WaitAck() == 0xff) {
        printf("error 4\r\n");
        return 0xff;
    }

    IIC_WriteBit(reg_addr);
    if (IIC_WaitAck() == 0xff) {
        printf("error 5\r\n");
        return 0xff;
    }

    IIC_WriteBit(data);
    if (IIC_WaitAck() == 0xff) {
        printf("error 6\r\n");
        return 0xff;
    }

    IIC_Stop();
    return 0;
}

int IIC_ReadData(unsigned char dev_addr, unsigned char reg_addr, unsigned char *pdata, unsigned char count) {
    unsigned char i;

    if (IIC_Start() == 0xff) {
        printf("error 7\r\n");
        return 0xff;
    }

    IIC_WriteBit(dev_addr);
    if (IIC_WaitAck() == 0xff) {
        // printf("error 8\r\n");
        return 0xff;
    }

    IIC_WriteBit(reg_addr);
    if (IIC_WaitAck() == 0xff) {
        printf("error 9\r\n");
        return 0xff;
    }

    IIC_Start();

    IIC_WriteBit(dev_addr + 1);
    if (IIC_WaitAck() == 0xff) {
        printf("error 2H\r\n");
        return 0xff;
    }

    for (i = 0; i < (count - 1); i++) {

        *pdata = IIC_ReadBit();
        IIC_Ack(0);
        pdata++;
    }

    *pdata = IIC_ReadBit();
    IIC_Ack(1);

    IIC_Stop();

    return 0;
}
