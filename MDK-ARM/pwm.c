// pwm.c

#include "pwm.h"
#include <main.h>
#include "tim.h"
// 定义TIM_HandleTypeDef
//TIM_HandleTypeDef htim4;

// 内部使用的函数声明
//static void MX_TIM4_Init(uint32_t pwm_period, uint32_t pwm_pulse);

void PWM_Init(uint32_t pwm_period, uint32_t pwm_pulse)
{
    // 停止定时器以进行配置
    PWM_Stop();

    // 修改定时器的周期（自动重装载值）
    __HAL_TIM_SET_AUTORELOAD(&htim2, pwm_period);

    // 修改比较寄存器的值（脉冲宽度）
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_pulse);

    // 更新定时器的配置信息
    __HAL_TIM_SET_COUNTER(&htim2, 0); // 重置计数器

    // 重新启动定时器
    PWM_Start();
}


void PWM_Start(void)
{
    // 开始PWM输出
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
    {
        // PWM启动错误处理
        Error_Handler();
    }
}

void PWM_Stop(void)
{
    // 停止PWM输出
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

void PWM_SetDutyCycle(uint32_t duty)
{
    uint32_t pulse = (htim2.Init.Period + 1) * duty / 100 - 1;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
}

void PWM_SetFrequency(uint32_t frequency)
{
    uint32_t new_period = (8000000 / frequency) - 1;

    PWM_Stop();

    htim2.Init.Period = new_period;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }

    PWM_Start();
}


