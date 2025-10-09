/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"
#include "Status.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define Standard_foot 20 //规定单独一格迷宫的小车移动距离

extern uint32_t arm_flag;
float ms = 0;
uint32_t wait_finish = 0;
uint32_t wait_flag = 0;
float wait_sec = 0;
uint32_t arm_num = 0;

uint32_t timer[] = {0, 0, 0, 0};
uint32_t ii = 0;
TIM_HandleTypeDef *htim[] = {&htim1, &htim2, &htim3, &htim4};
uint32_t Stop_flag[] = {};
uint32_t Stop_All_flag = 0;
float Move_Time[] = {0, 0, 0};

unsigned char Digtal;
unsigned int Sensor_Front[] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned int Sensor_Right[] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned int Sensor_Left[] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char Sensor_Color[] = {0, 0, 0};
unsigned char Anolog[8] = {0};
unsigned char rx_buff[256] = {0};
unsigned char Normal[8] = {0};

enum Trace_Dir Trace_flag = Left;
uint32_t flag[] = {0, 0, 0, 0};
float time[] = {2, 2, 2, 2};
int get = 0;
float Car_Speed = 0.2;

PID_Typedef Car_pid;

extern volatile uint8_t Obs_index_front;
extern volatile uint8_t Obs_index_left;
extern volatile uint8_t Obs_index_right;

extern volatile float Obs_distance_front;
extern volatile float Obs_distance_left;
extern volatile float Obs_distance_right;
extern volatile uint32_t Obs_buff_front;
extern volatile uint32_t Obs_buff_left;
extern volatile uint32_t Obs_buff_right;
extern volatile uint32_t Obs_data_ready;
extern volatile uint32_t Obs_index;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_Delay(100);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_TIM5_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
  PID_Init(&Car_pid, 0.05, 0.002, 0, 0.7);
  HAL_TIM_PWM_Init(&htim1);
  HAL_TIM_PWM_Init(&htim2);
  HAL_TIM_PWM_Init(&htim3);
  HAL_TIM_PWM_Init(&htim4);
  HAL_TIM_PWM_Init(&htim5);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);

  HAL_I2C_MspInit(&hi2c1);
  HAL_I2C_MspInit(&hi2c2);

  Ultrasonic_Init();

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    Get_Sensor(Sensor_Front);
    Get_Sensor(Sensor_Right);
    Get_Sensor(Sensor_Left);
    Get_Front_dis();//获取前方向超声波传感器的值，储存在Obs_distance_front
    Get_Left_dis();//获取左边超声波传感器的值，储存在Obs_distance_left
    Get_Left_dis();//获取右边超声波传感器的值，储存在Obs_distance_right
    Obstacle_find();
    //		Go(Move_Time[0],Move_Time[1],Move_Time[2]);
    HAL_Delay(30);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int wait_fun(float ms_num)
{
  if (wait_finish == 0)
  {
    wait_finish = 1;
    wait_flag = 1;
    ms = ms_num;
  }
  if (wait_finish == 2)
  {
    wait_finish = 0;
  }
  if (wait_flag == 0)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void wait_ms(float ms)
{
  if (wait_flag == 1)
  {
    wait_sec = ms * 100;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim5)
  {
    if (arm_flag == 1)
    {
      arm_num++;
    }
  }
}
// 前方向超声波测距
void Get_Front_dis(void)
{
  __HAL_TIM_SET_COUNTER(&htim8, 0);
    __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_CC1);
    __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_CC2);

    HAL_TIM_IC_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_IC_Start(&htim8, TIM_CHANNEL_2);
    Ultrasonic_Trig_front();
    
    Obs_index_front = 0;
    uint32_t expireTime = HAL_GetTick() + 50;
    while (expireTime > HAL_GetTick())
    {
      uint32_t CC1Flag = __HAL_TIM_GET_FLAG(&htim8, TIM_FLAG_CC1);
      uint32_t CC2Flag = __HAL_TIM_GET_FLAG(&htim8, TIM_FLAG_CC2);

      if (CC1Flag && CC2Flag)
      {
        Obs_index_front = 1;
        break;
      }
    }

    HAL_TIM_IC_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIM_IC_Stop(&htim8, TIM_CHANNEL_2);

    if (Obs_index_front == 1)
    {
      uint16_t ccr1 = __HAL_TIM_GET_COMPARE(&htim8, TIM_CHANNEL_1);
      uint16_t ccr2 = __HAL_TIM_GET_COMPARE(&htim8, TIM_CHANNEL_2);

      float pulseWidth = (ccr2 - ccr1)*1e-6f;
      Obs_distance_front = 1000 * 340.0f * pulseWidth / 2.0f;
    } 
}
// 左边超声波测距
void Get_Left_dis(void)
{
  __HAL_TIM_SET_COUNTER(&htim9, 0);
    __HAL_TIM_CLEAR_FLAG(&htim9, TIM_FLAG_CC1);
    __HAL_TIM_CLEAR_FLAG(&htim9, TIM_FLAG_CC2);

    HAL_TIM_IC_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_IC_Start(&htim9, TIM_CHANNEL_2);
    Ultrasonic_Trig_left();
    
    Obs_index_left = 0;
    uint32_t expireTime = HAL_GetTick() + 50;
    while (expireTime > HAL_GetTick())
    {
      uint32_t CC1Flag = __HAL_TIM_GET_FLAG(&htim9, TIM_FLAG_CC1);
      uint32_t CC2Flag = __HAL_TIM_GET_FLAG(&htim9, TIM_FLAG_CC2);

      if (CC1Flag && CC2Flag)
      {
        Obs_index_left = 1;
        break;
      }
    }

    HAL_TIM_IC_Stop(&htim9, TIM_CHANNEL_1);
    HAL_TIM_IC_Stop(&htim9, TIM_CHANNEL_2);

    if (Obs_index_left == 1)
    {
      uint16_t ccr1 = __HAL_TIM_GET_COMPARE(&htim9, TIM_CHANNEL_1);
      uint16_t ccr2 = __HAL_TIM_GET_COMPARE(&htim9, TIM_CHANNEL_2);

      float pulseWidth = (ccr2 - ccr1)*1e-6f;
      Obs_distance_left = 1000 * 340.0f * pulseWidth / 2.0f;
    } 
}
// 右边超声波测距
void Get_right_dis(void)
{
  __HAL_TIM_SET_COUNTER(&htim12, 0);
    __HAL_TIM_CLEAR_FLAG(&htim12, TIM_FLAG_CC1);
    __HAL_TIM_CLEAR_FLAG(&htim12, TIM_FLAG_CC2);

    HAL_TIM_IC_Start(&htim12, TIM_CHANNEL_1);
    HAL_TIM_IC_Start(&htim12, TIM_CHANNEL_2);
    Ultrasonic_Trig_left();
    
    Obs_index_left = 0;
    uint32_t expireTime = HAL_GetTick() + 50;
    while (expireTime > HAL_GetTick())
    {
      uint32_t CC1Flag = __HAL_TIM_GET_FLAG(&htim12, TIM_FLAG_CC1);
      uint32_t CC2Flag = __HAL_TIM_GET_FLAG(&htim12, TIM_FLAG_CC2);

      if (CC1Flag && CC2Flag)
      {
        Obs_index_left = 1;
        break;
      }
    }

    HAL_TIM_IC_Stop(&htim12, TIM_CHANNEL_1);
    HAL_TIM_IC_Stop(&htim12, TIM_CHANNEL_2);

    if (Obs_index_left == 1)
    {
      uint16_t ccr1 = __HAL_TIM_GET_COMPARE(&htim12, TIM_CHANNEL_1);
      uint16_t ccr2 = __HAL_TIM_GET_COMPARE(&htim12, TIM_CHANNEL_2);

      float pulseWidth = (ccr2 - ccr1)*1e-6f;
      Obs_distance_right = 1000 * 340.0f * pulseWidth / 2.0f;
    } 
}
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
