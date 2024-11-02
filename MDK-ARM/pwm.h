// pwm.h

#ifndef __PWM_H
#define __PWM_H

#include "stm32f1xx_hal.h"

// ��������PWM��TIM���
extern TIM_HandleTypeDef htim4;

// PWM��ʼ������������Ĭ�ϵ����ں�ռ�ձ�
void PWM_Init(uint32_t pwm_period, uint32_t pwm_pulse);

// ����PWMռ�ձȣ�dutyȡֵ0-100����ʾ�ٷֱ�
void PWM_SetDutyCycle(uint32_t duty);

// ����PWMƵ�ʣ�frequencyΪ�����Ƶ��ֵ��Hz��
void PWM_SetFrequency(uint32_t frequency);

// ����PWM���
void PWM_Start(void);

// ֹͣPWM���
void PWM_Stop(void);

#endif // __PWM_H