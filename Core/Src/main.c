/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "buzzer.h"
#include "pwm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED1_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_0

#define LED2_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_1

#define LED3_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_2

#define LED4_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_3

#define LED5_GPIO_Port GPIOA
#define LED5_Pin GPIO_PIN_4

#define LED6_GPIO_Port GPIOA
#define LED6_Pin GPIO_PIN_5

#define LED7_GPIO_Port GPIOA
#define LED7_Pin GPIO_PIN_6

#define LED8_GPIO_Port GPIOA
#define LED8_Pin GPIO_PIN_7

#define LED9_GPIO_Port GPIOB
#define LED9_Pin GPIO_PIN_0

#define LED10_GPIO_Port GPIOB
#define LED10_Pin GPIO_PIN_1	//LED initialization
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t HighCounts = 0;
uint32_t LowCounts  = 0;
uint8_t  IsHigh     = 0;//��ʱ���жϱ�־λ
uint32_t Counter 		= 0;
// �趨�ߵ�ƽ�͵͵�ƽ����ʱ�䣨��λ�����룩
uint32_t high_time_ms = 500; // �ߵ�ƽ����ʱ�䣬��λΪ����
uint32_t low_time_ms  = 500; // �͵�ƽ����ʱ�䣬��λΪ����
volatile uint32_t elapsed_time = 0; // ���ڴ洢������ʱ��
uint32_t timeInterval = 0;
const uint16_t velocity = 340;
uint8_t i = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}
//GPIO Pin initialization
void GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();  
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin	| LED5_Pin | LED6_Pin	|	LED7_Pin | LED8_Pin | LED9_Pin | LED10_Pin ;  
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);
		HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);
		HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);
		HAL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);
		HAL_GPIO_Init(LED5_GPIO_Port, &GPIO_InitStruct);
		HAL_GPIO_Init(LED6_GPIO_Port, &GPIO_InitStruct);
		HAL_GPIO_Init(LED7_GPIO_Port, &GPIO_InitStruct);
		HAL_GPIO_Init(LED8_GPIO_Port, &GPIO_InitStruct);
		HAL_GPIO_Init(LED9_GPIO_Port, &GPIO_InitStruct);
		HAL_GPIO_Init(LED10_GPIO_Port, &GPIO_InitStruct);
		
}

//Light LEDs
void light_up_leds(int num_leds) {
    if (num_leds >= 1) HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    if (num_leds >= 2) HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    if (num_leds >= 3) HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    if (num_leds >= 4) HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
    if (num_leds >= 5) HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
		if (num_leds >= 6) HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
		if (num_leds >= 7) HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_SET);
		if (num_leds >= 8) HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_SET);
		if (num_leds >= 9) HAL_GPIO_WritePin(LED9_GPIO_Port, LED9_Pin, GPIO_PIN_SET);
		if (num_leds >= 10) HAL_GPIO_WritePin(LED10_GPIO_Port, LED10_Pin, GPIO_PIN_SET);
}

//Turn off LEDs
void turn_off_all_leds(void) {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED9_GPIO_Port, LED9_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED10_GPIO_Port, LED10_Pin, GPIO_PIN_RESET);
}

// ��ʱ���ε�������LED
void light_up_leds_one_by_one(void) {
    // ���������ε���LED���ӳ�200ms��Ϊ��̬Ч��
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    delay_ms(200);

    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    delay_ms(200);

    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    delay_ms(200);

    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
    delay_ms(200);

		HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
    delay_ms(200);
		
		HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
    delay_ms(200);
	
		HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_SET);
    delay_ms(200);
		
		HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_SET);
    delay_ms(200);
		
		HAL_GPIO_WritePin(LED9_GPIO_Port, LED9_Pin, GPIO_PIN_SET);
    delay_ms(200);
		
		HAL_GPIO_WritePin(LED10_GPIO_Port, LED10_Pin, GPIO_PIN_SET);
    delay_ms(200);
}

//Blink LEDs
void blink_all_leds(uint32_t frequency) {
    // 4HzƵ����˸����˸һ������Ϊ250ms
    uint32_t period = 1000 / frequency / 2;  // ��������˸ʱ��
    
    while (1) {
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
      turn_off_all_leds();  // ȫ��LEDϨ��
			HAL_Delay(period);    // �ӳٰ������

      light_up_leds(10);     // ȫ��LED������������4��LED��
			HAL_Delay(period);    // �ӳٰ������
    }
}

//�����߼�
void control_leds(uint8_t n) {
    if (n == 0) {
        // ��ʱ���ε���LED
        light_up_leds_one_by_one();
    } else if (n >= 1 && n <= 9) {
        // ������1����n��LED
        turn_off_all_leds();  // ��Ϩ������LED
        light_up_leds(n);     // ����ָ��������LED
    } else if (n == 10) {
        // ��������LED����4HzƵ����˸
        blink_all_leds(4);
    }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void CalculateCounts(void)
{
    // ��ʱ���ж����ڣ���λΪ����
    uint32_t timer_period_ms = 1;

    // ����ߵ�ƽ�͵͵�ƽ��Ӧ�ļ���ֵ
    HighCounts = high_time_ms / timer_period_ms;
    LowCounts  = low_time_ms / timer_period_ms;

    // ȷ������ֵ��Ϊ��
    if (HighCounts == 0)
        HighCounts = 1;
    if (LowCounts == 0)
        LowCounts = 1;
}

// ��ʼ��ʱ����
void Start_Timer_Measurement(void)
{
  // ֹͣ��ʱ����ȷ����������ȷ����
  HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);

  // ���ü�����
  __HAL_TIM_SET_COUNTER(&htim3, 0);

  // ������벶���־λ������α����
  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC1);

  // ������ʱ�����������벶���ж�
  if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        Counter++; // ���Ӽ�����

        if (IsHigh)
        {
            if (Counter >= HighCounts)
            {
                // �ߵ�ƽ����ʱ���ѵ����л�Ϊ�͵�ƽ
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
                IsHigh  = 0;
                Counter = 0; // ���ü�����
            }
        }
        else
        {
            if (Counter >= LowCounts)
            {
                // �͵�ƽ����ʱ���ѵ����л�Ϊ�ߵ�ƽ
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
                IsHigh  = 1;
                Counter = 0; // ���ü�����
            }
        }
    }
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    // ֹͣ��ʱ�������������Ҫ��
    HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);

    // ��ȡ����ļ���ֵ
    elapsed_time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    // ����� elapsed_time ��Ϊ�ӿ�ʼ��ʱ�����벶���¼�֮���ʱ�䣨��λ��΢�룩
		timeInterval=elapsed_time/1000;
    // �������ڴ˴����� elapsed_time���������ñ�־λ�򴫵ݸ���ѭ��
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();
	HAL_ADCEx_Calibration_Start(&hadc1);
	Buzzer_SetFrequency(40000);
	HAL_ADC_Start(&hadc1);
  uint32_t adcValue = HAL_ADC_GetValue(&hadc1);
	float temperature = ((1.43 - ((float)adcValue * 3.3 / 4096)) / 0.0043) +24-320;
            // 1.43V is the typical voltage at 25��C
            // 0.0043 V/��C is the slope
	//Buzzer_Beep(150);
	Buzzer_Off();
	OLED_Clear();
  uint8_t n = 0;
		//control_leds(n);
	OLED_ShowString(1,1,"DST ");
	OLED_ShowString(2,1,"TMP ");
	OLED_ShowNum(2,5,temperature,3);
	high_time_ms=1;
	low_time_ms=1;//��Ҳ��֪��Ϊʲô��������ôд���С������1=7.2ms
	CalculateCounts();
	HAL_TIM_Base_Start_IT(&htim2);
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
 while (1)
  {
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		Buzzer_Beep(1);//1����40������
		Start_Timer_Measurement();
		HAL_Delay(125);
    while(TIM_FLAG_CC1==0&&i<=100)
		{
			i++;
		}
		float dst=velocity*timeInterval/2000;
		OLED_ShowNum(1,5,dst,4);
		/*if(dst<=40){n=10;}
		else if(40<dst&&dst<=45){n=9;}
		else if(45<dst&&dst<=50){n=8;}
		else if(50<dst&&dst<=55){n=7;}
		else if(55<dst&&dst<=60){n=6;}
		else if(60<dst&&dst<=70){n=5;}
		else if(70<dst&&dst<=80){n=4;}
		else if(80<dst&&dst<=90){n=3;}
		else if(90<dst&&dst<=100){n=2;}
		else{n=1;}
		*/
		HAL_Delay(150);
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
		HAL_Delay(1000);
    /* USER CODE END WHILE */
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
