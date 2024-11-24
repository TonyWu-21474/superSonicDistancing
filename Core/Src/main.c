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
#include "stdio.h"
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

#define LED8_GPIO_Port GPIOB
#define LED8_Pin GPIO_PIN_2

#define LED9_GPIO_Port GPIOB
#define LED9_Pin GPIO_PIN_10

#define LED10_GPIO_Port GPIOB
#define LED10_Pin GPIO_PIN_11	//LED initialization
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//不敢动但是也没用到
uint32_t HighCounts = 0;
uint32_t LowCounts  = 0;
uint8_t  IsHigh     = 0;//定时器中断标志位
uint32_t Counter 		= 0;
uint32_t counter1   = 0;
//蜂鸣器输出配置
// 设定高电平和低电平持续时间（单位：毫秒）
uint32_t high_time_ms = 500; // 高电平持续时间，单位为毫秒
uint32_t low_time_ms  = 500; // 低电平持续时间，单位为毫秒
//测距用
volatile uint32_t elapsed_time = 0; // 用于存储经过的时间
float timeInterval = 0;
uint16_t velocity = 340;
static float dst = 0;
static float dst0 = 0;
//超时计数器
uint32_t i = 0;
uint8_t flag=0;
uint8_t flag_led = 0;
uint8_t n = 0;
uint8_t flag_NoSig = 1;
/* 脉冲计数变量 */
volatile uint32_t pulse_count = 0;    // 已经输出的脉冲计数
volatile uint32_t pulse_target = 0;   // 需要输出的脉冲总数
uint16_t temp;
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

// 延时依次点亮所有LED
void light_up_leds_one_by_one(void) {
    // 从左到右依次点亮LED，延迟200ms作为动态效果
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED9_GPIO_Port, LED9_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED10_GPIO_Port, LED10_Pin, GPIO_PIN_SET);
}

//Blink LEDs
void blink_all_leds(uint32_t frequency) {
    // 4Hz频率闪烁，闪烁一次周期为250ms
    uint32_t period = 1000 / frequency / 2;  // 半周期闪烁时间
    
    while (1) {
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
      turn_off_all_leds();  // 全部LED熄灭
			HAL_Delay(period);    // 延迟半个周期

      light_up_leds(10);     // 全部LED点亮（假设有4个LED）
			HAL_Delay(period);    // 延迟半个周期
    }
}

//控制逻辑
void control_leds(uint8_t n) {
    if (n == 0) {
        // 延时依次点亮LED
        light_up_leds_one_by_one();
    } else if (n >= 1 && n <= 9) {
        // 点亮第1到第n个LED
        turn_off_all_leds();  // 先熄灭所有LED
        light_up_leds(n);     // 点亮指定数量的LED
    } else if (n == 10) {
        light_up_leds(n);
    }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void CalculateCounts(void)
{
    // 定时器中断周期，单位为毫秒
    uint32_t timer_period_ms = 1;

    // 计算高电平和低电平对应的计数值
    HighCounts = high_time_ms / timer_period_ms;
    LowCounts  = low_time_ms / timer_period_ms;

    // 确保计数值不为零
    if (HighCounts == 0)
        HighCounts = 1;
    if (LowCounts == 0)
        LowCounts = 1;
}

// 开始计时函数
void Start_Timer_Measurement(void)
{

  // 停止定时器以确保计数器正确重置
  HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);

  // 重置计数器
  __HAL_TIM_SET_COUNTER(&htim3, 0);

  // 清除输入捕获标志位，避免伪触发
  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC1);

  // 启动定时器计数和输入捕获中断
  if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)//没写完，IO外部中断用于定时
	//1107更新：可以读取寄存器值，需要进一步修改参数
{
	flag = 1;//只更新标志位
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)	//定时器2/4溢出中断处理,未测试
																														//1107测试结果：正常
{
    if (htim->Instance == TIM2)
    {
        Counter++; // 增加计数器

        if (IsHigh)
        {
            if (Counter >= HighCounts)
            {
                // 高电平持续时间已到，切换为低电平
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
                IsHigh  = 0;
                Counter = 0; // 重置计数器
            }
        }
        else
        {
            if (Counter >= LowCounts)
            {
                // 低电平持续时间已到，切换为高电平
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
                IsHigh  = 1;
                Counter = 0; // 重置计数器
            }
        }
    }
		if (htim ->Instance == TIM4)
    {
        pulse_count++;
        if (pulse_count >= pulse_target)
        {
            /* 达到目标脉冲数，停止PWM输出 */
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
            
            /* 禁用TIM4更新中断 */
            __HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE);
        }
    }
		if (htim->Instance == TIM1) 
		{
      if (flag_led == 0 && n == 10)  
			{
				light_up_leds(10);
				flag_led = 1;
			}
			else if(flag_led == 1 && n == 10)
			{
				turn_off_all_leds();
				flag_led = 0;
			}
    }
}


/*void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    // 停止定时器计数（如果需要）
    HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);

    // 读取捕获的计数值
    elapsed_time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    // 这里的 elapsed_time 即为从开始计时到输入捕获事件之间的时间（单位：微秒）
		timeInterval=elapsed_time/1000;
    // 您可以在此处处理 elapsed_time，例如设置标志位或传递给主循环
  }
}
*/

void beep(uint8_t peaks)
{
		pulse_target = peaks;
		pulse_count = 0;
		HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
		__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
//		HAL_Delay(1);
//		HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
//		HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_2);
}
void float_to_string_simple(float num, char *str, int precision) 
{
    if (num < 0) { // 处理负数
        *str++ = '-';
        num = -num;
    }

    int int_part = (int)num; // 整数部分
    float frac_part = num - int_part; // 小数部分

    // 将整数部分转换为字符串
    sprintf(str, "%d", int_part);
    while (*str != '\0') str++; // 移动到字符串末尾

    // 添加小数点
    *str++ = '.';

    // 处理小数部分
    for (int i = 0; i < precision; i++) {
        frac_part *= 10;
        int digit = (int)frac_part;
        *str++ = '0' + digit;
        frac_part -= digit;
    }
    *str = '\0'; // 结束符
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	//light_up_leds(10);
	//HAL_NVIC_DisableIRQ(EXTI4_IRQn);
	n = 0;
	
	OLED_Init();
	HAL_ADCEx_Calibration_Start(&hadc1);
	//Buzzer_SetFrequency(40000);
	HAL_ADC_Start(&hadc1);
  uint16_t adcValue = HAL_ADC_GetValue(&hadc1);
	float temperature = (1.43-(float)adcValue * 3.3f / 4096.0f)/0.0043f -25.0f-70.0f;
            // 1.43V is the typical voltage at 25°C
            // 0.0043 V/°C is the slope
	//Buzzer_Beep(150);
	//Buzzer_Off();
	//float_to_string_simple(temperature,temp,2);
	OLED_Clear();
  //uint8_t n = 0;
		//control_leds(n);
	OLED_ShowString(1,1,"DST ");
	OLED_ShowString(2,1,"TMP ");
	OLED_ShowString(3,1,"SPD ");
	OLED_ShowNum(2,5,temperature,4);
	//OLED_ShowString(2,5,temp);
	OLED_ShowNum(3,5,velocity,3);
	high_time_ms=0;
	low_time_ms=400; //时钟要配置为72MHz
	CalculateCounts();
		HAL_TIM_Base_Start_IT(&htim1);
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
	pulse_target = 9;
  pulse_count = 0;
	HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	i=0;
	//velocity = velocity / 10.0f;
	uint16_t j = 2;//暂时存储发波数
	counter1 = 0;
	temp=0;
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
	//EXTI->FTSR |= 1<<4;
	//temp = EXTI->FTSR; //TODO :显示标志位值
	//OLED_ShowHexNum(4,5,temp,8);
	while (1)
  {
		//if(dst0-dst>=5){j++;}
		//else if(dst-dst0>=5){j--;}
		//else{}
		
		dst0=dst;
		OLED_ShowNum(4,1,j,3);
		//Buzzer_Beep(0);//1毫秒40个波形，考虑弃用该部分
		flag = 0;												//清中断标志位
		__HAL_TIM_SET_COUNTER(&htim3,0);//清计数器
		HAL_NVIC_DisableIRQ(EXTI4_IRQn);
		HAL_TIM_Base_Start(&htim3);
		beep(j);//发信号
		//OLED_ShowNum(4,1,i,2);
		//HAL_Delay(1);
		i=0;
		
		i = 0;
//		while(i<=59000)
//		{
//			i++;//等待余震
//			//temp++;
//		}
//		//__HAL_TIM_SET_COUNTER(&htim3,0);
		timeInterval = __HAL_TIM_GET_COUNTER(&htim3);
		HAL_NVIC_EnableIRQ(EXTI4_IRQn);
		timeInterval = __HAL_TIM_GET_COUNTER(&htim3);
		
		i = 0;
		if(HAL_TIM_Base_GetState(&htim3) != HAL_TIM_STATE_BUSY)
		{
			OLED_Clear();
			OLED_ShowString(1,1,"ERROR!");
			Error_Handler();
		}
//=======
//		//HAL_NVIC_DisableIRQ(EXTI4_IRQn);//先关闭中断，防止错误触发
//		//if(dst0-dst>=5){j++;}
//		//else if(dst-dst0>=5){j--;}
//		//else{}
//		j++;
//		dst0=dst;
//		OLED_ShowNum(4,1,j,2);
//		//HAL_NVIC_DisableIRQ(EXTI4_IRQn);//先关闭中断，防止错误触发
//		//Buzzer_Beep(0);//1毫秒40个波形，考虑弃用该部分
//		flag = 0;												//清中断标志位
//		__HAL_TIM_SET_COUNTER(&htim3,0);
//		beep(6);//发信号
//		//OLED_ShowNum(4,1,i,2);
//		//HAL_Delay(1);
//		i=0;
//		HAL_TIM_Base_Start(&htim3);
//		__HAL_TIM_SET_COUNTER(&htim3,0);
//		while(i<=60330)
//		{
//			i++;//等待余震
//			temp++;
//		}
//		i = 0;
//		if(HAL_TIM_Base_GetState(&htim3) != HAL_TIM_STATE_BUSY)
//		{
//			OLED_Clear();
//			OLED_ShowString(1,1,"ERROR!");
//			Error_Handler();
//		}
		//flag = 0;
//>>>>>>> Stashed changes
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		while(flag !=1 && i<720000)
		{
			i++;//等待10ms超时
			temp++;
		}
		i=0;
//		//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_7);
//    //现在（11/4）的问题是在初始化完成并且启动定时器后，HAL_TIM_Base_GetState仍然返回0
//		//下一步向老师询问为什么出现这样的问题
//		//已解决（11/7）
		if(flag == 0x01)
		{
			
			//temp = EXTI->FTSR; //TODO :显示标志位值
			//OLED_ShowHexNum(4,5,temp,4);
			timeInterval = __HAL_TIM_GET_COUNTER(&htim3);
			__HAL_TIM_SET_COUNTER(&htim3,0);
			timeInterval = timeInterval/1000.0f;
			//timeInterval = timeInterval/100;
			counter1++;
			OLED_ShowNum(4,5,counter1,3);
			//HAL_GPIO_WritePin(GPIOA,7,GPIO_PIN_RESET);
			dst=velocity*timeInterval / 20.0f;
			//OLED_Clear();
			high_time_ms=1;
			dst0=dst;
			if(dst<=40){n=10;j=4;}
				else if(40<dst&&dst<=45){n=9;j=8;dst = dst -1;}
				else if(45<dst&&dst<=50){n=8;j=8;dst = dst -1 ;}
				else if(50<dst&&dst<=55){n=7;j=16;dst = dst - 3;}
				else if(55<dst&&dst<=60){n=6;j=16;dst = dst - 3;}
				else if(60<dst&&dst<=70){n=5;j=16;dst = dst - 3;}
				else if(70<dst&&dst<=80){n=4;j=32;dst = dst - 3;}
				else if(80<dst&&dst<=90){n=3;j=32;dst = dst - 3;}
				else if(90<dst&&dst<=100){n=2;j=64;dst = dst - 3;}
				else{n=1; high_time_ms = 0;}
				if(dst > 100 ){ dst = dst -10;}
				control_leds(n);
				OLED_ShowString(1,1,"                ");//清行
				OLED_ShowString(1,1,"DST ");
				//OLED_ShowString(2,1,"TMP ");
				//OLED_ShowString(3,1,"SPD ");
				OLED_ShowFloat(2,5,temperature,2);
				//OLED_ShowString(2,5,temp);
				OLED_ShowNum(3,5,velocity,3);
				OLED_ShowFloat(1,5,dst,5);
				control_leds(n);
				low_time_ms=1000/n-1; //时钟要配置为72MHz
				if(n == 10) {low_time_ms = 1;}
				CalculateCounts();
				HAL_TIM_Base_Stop(&htim3);
				__HAL_TIM_SET_COUNTER(&htim3,0);
				flag = 0;		//中断触发后清标志位
				flag_NoSig = 0;
			}
		else if (flag != 0x01) 
		{
				OLED_ShowString(1,1,"                ");
				OLED_ShowString(1,4,"No Signal!");
				HAL_TIM_Base_Stop(&htim3);
				__HAL_TIM_SET_COUNTER(&htim3,0);
			  high_time_ms = 0;
				CalculateCounts();
				flag_NoSig = 1;
		}
		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
		
		//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		adcValue = HAL_ADC_GetValue(&hadc1);
		temperature = (1.43-(float)adcValue * 3.3f / 4096.0f)/0.0043f -25.0f;
		OLED_ShowNum(2,5,temperature,2);
		if(temperature>5 && temperature<25){ velocity = 331.4 + 0.6 *temperature;}
		//OLED_ShowNum(4,5,adcValue,5);
		if(flag_NoSig == 1) {j=j*2;}
		else if(flag_NoSig == 0) {j = j;}
		if(j > 512 ) {j = 2;}
		HAL_Delay(250);
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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
