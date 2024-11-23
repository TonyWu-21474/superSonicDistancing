#include "stm32f1xx_hal.h"
#include "OLED_Font.h"
#include "gpio.h"
#include "stdio.h"
/* 引脚配置 */
#define OLED_SCL_GPIO_PORT   GPIOB
#define OLED_SCL_PIN         GPIO_PIN_8
#define OLED_SDA_GPIO_PORT   GPIOB
#define OLED_SDA_PIN         GPIO_PIN_9

/* 引脚操作宏 */
#define OLED_SCL_HIGH()      HAL_GPIO_WritePin(OLED_SCL_GPIO_PORT, OLED_SCL_PIN, GPIO_PIN_SET)
#define OLED_SCL_LOW()       HAL_GPIO_WritePin(OLED_SCL_GPIO_PORT, OLED_SCL_PIN, GPIO_PIN_RESET)
#define OLED_SDA_HIGH()      HAL_GPIO_WritePin(OLED_SDA_GPIO_PORT, OLED_SDA_PIN, GPIO_PIN_SET)
#define OLED_SDA_LOW()       HAL_GPIO_WritePin(OLED_SDA_GPIO_PORT, OLED_SDA_PIN, GPIO_PIN_RESET)

/* 引脚初始化 */
void OLED_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能 GPIO 时钟 */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 配置 SCL 引脚 */
    GPIO_InitStruct.Pin = OLED_SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(OLED_SCL_GPIO_PORT, &GPIO_InitStruct);

    /* 配置 SDA 引脚 */
    GPIO_InitStruct.Pin = OLED_SDA_PIN;
    HAL_GPIO_Init(OLED_SDA_GPIO_PORT, &GPIO_InitStruct);

    /* 初始化引脚状态 */
    OLED_SCL_HIGH();
    OLED_SDA_HIGH();
}

/* I2C 开始信号 */
void OLED_I2C_Start(void)
{
    OLED_SDA_HIGH();
    OLED_SCL_HIGH();
    //HAL_Delay(1);
    OLED_SDA_LOW();
    //HAL_Delay(1);
    OLED_SCL_LOW();
}

/* I2C 停止信号 */
void OLED_I2C_Stop(void)
{
    OLED_SCL_LOW();
    OLED_SDA_LOW();
    //HAL_Delay(1);
    OLED_SCL_HIGH();
    //HAL_Delay(1);
    OLED_SDA_HIGH();
}

/* I2C 发送一个字节 */
void OLED_I2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        if (Byte & 0x80)
            OLED_SDA_HIGH();
        else
            OLED_SDA_LOW();
        //HAL_Delay(1);
        OLED_SCL_HIGH();
        //HAL_Delay(1);
        OLED_SCL_LOW();
        //HAL_Delay(1);
        Byte <<= 1;
    }
    /* 发送 ACK 时钟 */
    OLED_SDA_HIGH();  // 释放 SDA 线，等待 ACK
    //HAL_Delay(1);
    OLED_SCL_HIGH();
    //HAL_Delay(1);
    OLED_SCL_LOW();
    //HAL_Delay(1);
}

/* OLED 写命令 */
void OLED_WriteCommand(uint8_t Command)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);    // 从机地址
    OLED_I2C_SendByte(0x00);    // 写命令
    OLED_I2C_SendByte(Command);
    OLED_I2C_Stop();
}

/* OLED 写数据 */
void OLED_WriteData(uint8_t Data)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);    // 从机地址
    OLED_I2C_SendByte(0x40);    // 写数据
    OLED_I2C_SendByte(Data);
    OLED_I2C_Stop();
}

/* OLED 设置光标位置 */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0xB0 + Y);                        // 设置页地址（Y 位置）
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));        // 设置列高位
    OLED_WriteCommand(0x00 | (X & 0x0F));               // 设置列低位
}

/* OLED 清屏 */
void OLED_Clear(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++)
    {
        OLED_SetCursor(j, 0);
        for (i = 0; i < 128; i++)
        {
            OLED_WriteData(0x00);
        }
    }
}

/* OLED 显示一个字符 */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t i;
    uint8_t c = Char - ' ';  // 偏移到字模数组中的位置

    OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[c][i]);  // 显示上半部分
    }
    OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[c][ i + 8]);  // 显示下半部分
    }
}
/* OLED显示浮点数 */


/* OLED 显示字符串 */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    uint8_t i = 0;
    while (String[i] != '\0')
    {
        OLED_ShowChar(Line, Column + i, String[i]);
        i++;
    }
}

/* OLED 次方函数 */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--)
    {
        Result *= X;
    }
    return Result;
}

/* OLED 显示整数（无符号） */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/* OLED 显示整数（有符号） */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint32_t Number1;
    uint8_t i;

    if (Number >= 0)
    {
        OLED_ShowChar(Line, Column, '+');
        Number1 = Number;
    }
    else
    {
        OLED_ShowChar(Line, Column, '-');
        Number1 = -Number;
    }

    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + 1 + i, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/* OLED 显示十六进制数 */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i, SingleNumber;
    for (i = 0; i < Length; i++)
    {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
        if (SingleNumber < 10)
        {
            OLED_ShowChar(Line, Column + i, SingleNumber + '0');
        }
        else
        {
            OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
        }
    }
}

/* OLED 显示二进制数 */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
    }
}
void OLED_ShowFloat(uint8_t Line , uint8_t Column, float Number, uint8_t intLength)
{
		uint32_t PowNum , IntNum , FraNum;
		IntNum = Number;
		Number -= IntNum;
		PowNum = 100;
		FraNum = PowNum * Number;
		OLED_ShowNum(Line,Column,IntNum,intLength);
		OLED_ShowChar(Line,Column+intLength,'.');
		OLED_ShowNum(Line,Column+intLength+1,FraNum,2);
}
/* OLED 初始化 */
void OLED_Init(void)
{
    HAL_Delay(100);  // 上电延时

    OLED_GPIO_Init();  // GPIO 初始化

    OLED_WriteCommand(0xAE);  // 关闭显示

    OLED_WriteCommand(0xD5);  // 设置显示时钟分频比/振荡器频率
    OLED_WriteCommand(0x80);

    OLED_WriteCommand(0xA8);  // 设置多路复用率
    OLED_WriteCommand(0x3F);

    OLED_WriteCommand(0xD3);  // 设置显示偏移
    OLED_WriteCommand(0x00);

    OLED_WriteCommand(0x40);  // 设置显示开始行

    OLED_WriteCommand(0xA1);  // 设置左右方向，0xA1 正常 0xA0 左右反置

    OLED_WriteCommand(0xC8);  // 设置上下方向，0xC8 正常 0xC0 上下反置

    OLED_WriteCommand(0xDA);  // 设置 COM 引脚硬件配置
    OLED_WriteCommand(0x12);

    OLED_WriteCommand(0x81);  // 设置对比度控制
    OLED_WriteCommand(0xCF);

    OLED_WriteCommand(0xD9);  // 设置预充电周期
    OLED_WriteCommand(0xF1);

    OLED_WriteCommand(0xDB);  // 设置 VCOMH 取消选择级别
    OLED_WriteCommand(0x30);

    OLED_WriteCommand(0xA4);  // 设置整个显示打开/关闭

    OLED_WriteCommand(0xA6);  // 设置正常/倒转显示

    OLED_WriteCommand(0x8D);  // 设置充电泵
    OLED_WriteCommand(0x14);

    OLED_WriteCommand(0xAF);  // 开启显示

    OLED_Clear();  // 清屏
}