// pwm.h

#ifndef __PWM_H
#define __PWM_H

#include "stm32f1xx_hal.h"

// 定义用于PWM的TIM句柄
extern TIM_HandleTypeDef htim4;

// PWM初始化函数，传入默认的周期和占空比
void PWM_Init(uint32_t pwm_period, uint32_t pwm_pulse);

// 设置PWM占空比，duty取值0-100，表示百分比
void PWM_SetDutyCycle(uint32_t duty);

// 设置PWM频率，frequency为所需的频率值（Hz）
void PWM_SetFrequency(uint32_t frequency);

// 启动PWM输出
void PWM_Start(void);

// 停止PWM输出
void PWM_Stop(void);

#endif // __PWM_H