/* buzzer.c */
#include "buzzer.h"

void Buzzer_Init(uint32_t frequency)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_OC_InitTypeDef sConfigOC1 = {0};
  /* ��ȡ��ʱ��ʱ��Ƶ�� */
  uint32_t uwTimClock = HAL_RCC_GetPCLK1Freq(); // TIM4 λ�� APB1

  /* ����ʱ����Ƶ */
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
  {
    uwTimClock *= 2;
  }

  /* ����Ԥ��Ƶֵ */
  uint32_t uwPrescalerValue = (uwTimClock / 1000000) - 1; // ��ʱ������Ƶ�� 1MHz

  /* �����Զ���װ��ֵ */
  uint32_t uwPeriod = (1000000 / frequency) - 1;

  /* ���ö�ʱ��Ԥ��Ƶ������ */
  htim4.Init.Prescaler = uwPrescalerValue;
  htim4.Init.Period = uwPeriod;

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    /* ��ʼ��ʧ�� */
    Error_Handler();
  }

  /* ���� PWM ͨ�� */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = (uwPeriod + 1) / 2; // ռ�ձ� 50%
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC1.OCMode = TIM_OCMODE_PWM2;
  sConfigOC1.Pulse = (uwPeriod + 1) / 2; // ռ�ձ� 50%
  sConfigOC1.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC1.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK || HAL_TIM_PWM_ConfigChannel(&htim4,&sConfigOC1,TIM_CHANNEL_2) != HAL_OK)
  {
    /* ����ʧ�� */
    Error_Handler();
  }

  /* ���� PWM ��� */
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK||HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2)!=HAL_OK)
  {
    /* ����ʧ�� */
    Error_Handler();
  }
}

void Buzzer_On(void)
{
	if(HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK||HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2)!=HAL_OK)
	{
		Error_Handler();
	}
  /* ��������ͨ�� PWM ��������������� */
}

void Buzzer_Off(void)
{
  /* ֹͣ PWM ��� */
  if (HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1) != HAL_OK || HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2)!=HAL_OK)
  {
    /* ֹͣʧ�� */
    Error_Handler();
  }
}

void Buzzer_SetFrequency(uint32_t frequency)
{
  /* ����������Ƶ�� */
  Buzzer_Init(frequency);
}
void Buzzer_Beep(uint32_t duration_ms)
{
  /* ���������� */
  Buzzer_On();
  HAL_Delay(duration_ms);
  Buzzer_Off();
}