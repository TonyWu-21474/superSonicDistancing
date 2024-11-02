// pwm.c

#include "pwm.h"
#include <main.h>
#include "tim.h"
// ����TIM_HandleTypeDef
//TIM_HandleTypeDef htim4;

// �ڲ�ʹ�õĺ�������
//static void MX_TIM4_Init(uint32_t pwm_period, uint32_t pwm_pulse);

void PWM_Init(uint32_t pwm_period, uint32_t pwm_pulse)
{
    // ֹͣ��ʱ���Խ�������
    PWM_Stop();

    // �޸Ķ�ʱ�������ڣ��Զ���װ��ֵ��
    __HAL_TIM_SET_AUTORELOAD(&htim2, pwm_period);

    // �޸ıȽϼĴ�����ֵ�������ȣ�
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_pulse);

    // ���¶�ʱ����������Ϣ
    __HAL_TIM_SET_COUNTER(&htim2, 0); // ���ü�����

    // ����������ʱ��
    PWM_Start();
}


void PWM_Start(void)
{
    // ��ʼPWM���
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
    {
        // PWM����������
        Error_Handler();
    }
}

void PWM_Stop(void)
{
    // ֹͣPWM���
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


