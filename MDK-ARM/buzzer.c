/* buzzer.c */
#include "buzzer.h"

void Buzzer_Init(uint32_t frequency)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_OC_InitTypeDef sConfigOC1 = {0};
  /* 获取定时器时钟频率 */
  uint32_t uwTimClock = HAL_RCC_GetPCLK1Freq(); // TIM4 位于 APB1

  /* 处理定时器倍频 */
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
  {
    uwTimClock *= 2;
  }

  /* 计算预分频值 */
  uint32_t uwPrescalerValue = (uwTimClock / 1000000) - 1; // 定时器计数频率 1MHz

  /* 计算自动重装载值 */
  uint32_t uwPeriod = (1000000 / frequency) - 1;

  /* 设置定时器预分频和周期 */
  htim4.Init.Prescaler = uwPrescalerValue;
  htim4.Init.Period = uwPeriod;

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    /* 初始化失败 */
    Error_Handler();
  }

  /* 配置 PWM 通道 */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = (uwPeriod + 1) / 2; // 占空比 50%
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC1.OCMode = TIM_OCMODE_PWM2;
  sConfigOC1.Pulse = (uwPeriod + 1) / 2; // 占空比 50%
  sConfigOC1.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC1.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK || HAL_TIM_PWM_ConfigChannel(&htim4,&sConfigOC1,TIM_CHANNEL_2) != HAL_OK)
  {
    /* 配置失败 */
    Error_Handler();
  }

  /* 启动 PWM 输出 */
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK||HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2)!=HAL_OK)
  {
    /* 启动失败 */
    Error_Handler();
  }
}

void Buzzer_On(void)
{
	if(HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK||HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2)!=HAL_OK)
	{
		Error_Handler();
	}
  /* 蜂鸣器已通过 PWM 输出，无需额外操作 */
}

void Buzzer_Off(void)
{
  /* 停止 PWM 输出 */
  if (HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1) != HAL_OK || HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2)!=HAL_OK)
  {
    /* 停止失败 */
    Error_Handler();
  }
}

void Buzzer_SetFrequency(uint32_t frequency)
{
  /* 调整蜂鸣器频率 */
  Buzzer_Init(frequency);
}
void Buzzer_Beep(uint32_t duration_ms)
{
  /* 蜂鸣器发声 */
  Buzzer_On();
  HAL_Delay(duration_ms);
  Buzzer_Off();
}