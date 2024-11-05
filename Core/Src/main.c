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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "state.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const float LIMIT_ANGLE_H=1.8;
const float LIMIT_ANGLE_V=1.0;
const float T = 0.05; //補間時間 プリスケーラ9000 コンペア500
const int PULSE_LIMIT=40000;

uint8_t dma_rx_buff[1];
uint8_t rx_buff[128];
uint8_t tx_buff[128];

uint16_t g_ADCBuffer[4];
uint16_t link1_adc=0;
uint16_t link2_adc=0;
uint16_t link3_adc=0;
uint16_t link4_adc=0;
uint8_t grab_pin=0;

State state = {
	0,
	0.0,
	0.0,
	30.0,
	0.0,
	0,
	0
};

State now_state = {
	0,
	0.0,
	0.0,
	30.0,
	0.0,
	0,
	0
};

State tgt_state = {
	0,
	0.0,
	0.0,
	30.0,
	0.0,
	0,
	0
};

State pre_state = {
	0,
	0.0,
	0.0,
	30.0,
	0.0,
	0,
	0
};

uint8_t link1_str[6];
uint8_t link2_str[6];
uint8_t link3_str[6];
uint8_t link4_str[6];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int deg2pulse(float deg)
{
	int pulse = (int)(3000.0+2000.0/90.0*deg);
	if (pulse>=1000 && pulse <=5000)
	{
		return PULSE_LIMIT-pulse;
	}
	else
	{
		return PULSE_LIMIT-3000;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
    	if (state.counter<128){
    		if(dma_rx_buff[0]=='\n'){
    			rx_buff[state.counter]=dma_rx_buff[0];
    			UpdateTarget();
    			state.counter=0;
    			memset(rx_buff,0,sizeof(rx_buff));
    		} else {
    			rx_buff[state.counter]=dma_rx_buff[0];
    			state.counter++;
    		}
    	} else {
    		state.counter=0;
    		memset(rx_buff,0,sizeof(rx_buff));
    	}
        HAL_UART_Receive_IT(&huart2, dma_rx_buff, 1);
    }
}

void UpdateTarget(){
	if (
		rx_buff[0]=='A' &&
		rx_buff[7]=='B' &&
		rx_buff[14]=='C' &&
		rx_buff[21]=='D' &&
		rx_buff[28]=='G' &&
		rx_buff[30]=='M'
	){
		sprintf(
			link1_str,
			"%c%c%c%c%c%c",
			rx_buff[1],
			rx_buff[2],
			rx_buff[3],
			rx_buff[4],
			rx_buff[5],
			rx_buff[6]
		);
		state.link1=atof(link1_str);
		sprintf(
			link2_str,
			"%c%c%c%c%c%c",
			rx_buff[8],
			rx_buff[9],
			rx_buff[10],
			rx_buff[11],
			rx_buff[12],
			rx_buff[13]
		);
		state.link2=atof(link2_str);
		sprintf(
			link3_str,
			"%c%c%c%c%c%c",
			rx_buff[15],
			rx_buff[16],
			rx_buff[17],
			rx_buff[18],
			rx_buff[19],
			rx_buff[20]
		);
		state.link3=atof(link3_str);
		sprintf(
			link4_str,
			"%c%c%c%c%c%c",
			rx_buff[22],
			rx_buff[23],
			rx_buff[24],
			rx_buff[25],
			rx_buff[26],
			rx_buff[27]
		);
		state.link4=atof(link4_str);
		state.glab = rx_buff[29];
		state.mode=rx_buff[31];
		if (state.glab=='1')
		{
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,40000-1500); //1000(0.5ms)~3000(1.5ms)~5000(2.5ms)
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,40000-3000); //1000(0.5ms)~3000(1.5ms)~5000(2.5ms)
		}
	}
	else
	{
	}
}

float update_tgt_h(float state_link,float tgt_link)
{
	if (abs(state_link-tgt_link)>LIMIT_ANGLE_H && (state_link-tgt_link)>0)
	{
		return tgt_link+LIMIT_ANGLE_H;
	}
	else if (abs(state_link-tgt_link)>LIMIT_ANGLE_H && (state_link-tgt_link)<0)
	{
		return tgt_link-LIMIT_ANGLE_H;
	}

	return state_link;
}

float update_tgt_v(float state_link,float tgt_link)
{
	if (abs(state_link-tgt_link)>LIMIT_ANGLE_V && (state_link-tgt_link)>0)
	{
		return tgt_link+LIMIT_ANGLE_V;
	}
	else if (abs(state_link-tgt_link)>LIMIT_ANGLE_V && (state_link-tgt_link)<0)
	{
		return tgt_link-LIMIT_ANGLE_V;
	}

	return state_link;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	static float t = 0.0f;

	if(htim == &htim6) {
		now_state.link1=
				-2.0*(tgt_state.link1-pre_state.link1)*powf(t,3.0)/powf(T,3.0)+
				3.0*(tgt_state.link1-pre_state.link1)*powf(t,2.0)/powf(T,2.0)+
				pre_state.link1;
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,deg2pulse(now_state.link1)); //1000(0.5ms)~3000(1.5ms)~5000(2.5ms)

		now_state.link2=
				-2.0*(tgt_state.link2-pre_state.link2)*powf(t,3.0)/powf(T,3.0)+
				3.0*(tgt_state.link2-pre_state.link2)*powf(t,2.0)/powf(T,2.0)+
				pre_state.link2;
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,deg2pulse(now_state.link2)); //1000(0.5ms)~3000(1.5ms)~5000(2.5ms)

		now_state.link3=
				-2.0*(tgt_state.link3-pre_state.link3)*powf(t,3.0)/powf(T,3.0)+
				3.0*(tgt_state.link3-pre_state.link3)*powf(t,2.0)/powf(T,2.0)+
				pre_state.link3;
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,deg2pulse(now_state.link3)); //1000(0.5ms)~3000(1.5ms)~5000(2.5ms)

		now_state.link4=
				-2.0*(tgt_state.link4-pre_state.link4)*powf(t,3.0)/powf(T,3.0)+
				3.0*(tgt_state.link4-pre_state.link4)*powf(t,2.0)/powf(T,2.0)+
				pre_state.link4;
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,deg2pulse(now_state.link4)); //1000(0.5ms)~3000(1.5ms)~5000(2.5ms)

		//t+=0.005;
		t +=T*0.02;
	}

    if(htim == &htim7) {
		sprintf(
			tx_buff,
			"{\"A\":%d,\"B\":%d,\"C\":%d,\"D\":%d,\"G\":%d,\"M\":%c}\n",
			link1_adc,
			link2_adc,
			link3_adc,
			link4_adc,
			grab_pin,
			state.mode
		);
  		HAL_UART_Transmit(&huart2,tx_buff,strlen(tx_buff),20);
		memset(tx_buff,0,sizeof(tx_buff));
    	t=0.0;
  		pre_state=tgt_state;

  		//tgt_state = state;

  		tgt_state.link1 = update_tgt_h(state.link1, tgt_state.link1);
  		tgt_state.link2 = update_tgt_v(state.link2, tgt_state.link2);
  		tgt_state.link3 = update_tgt_v(state.link3, tgt_state.link3);
  		tgt_state.link4 = update_tgt_h(state.link4, tgt_state.link4);
  		tgt_state.glab = state.glab;
  		tgt_state.mode = state.mode;
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,deg2pulse(0.0)); //1000(0.5ms)~3000(1.5ms)~5000(2.5ms)
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,deg2pulse(0.0)); //1000(0.5ms)~3000(1.5ms)~5000(2.5ms)
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,deg2pulse(50.0)); //1000(0.5ms)~3000(1.5ms)~5000(2.5ms)
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,deg2pulse(0.0)); //1000(0.5ms)~3000(1.5ms)~5000(2.5ms)
  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,deg2pulse(0.0)); //1000(0.5ms)~3000(1.5ms)~5000(2.5ms)
  HAL_UART_Receive_IT(&huart2, dma_rx_buff, 1);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  memset(g_ADCBuffer, 0, sizeof(g_ADCBuffer));
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_ADCBuffer, 4);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if( __HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE))
	  {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		HAL_NVIC_DisableIRQ(USART2_IRQn);
		__disable_irq();

		//一応フラグをクリアする
		__HAL_UART_CLEAR_NEFLAG(&huart2);
		__HAL_UART_CLEAR_OREFLAG(&huart2);

		//UARTを初期化する．DeInitいらない説
		HAL_UART_DeInit(&huart2);
		MX_USART2_UART_Init();

		//再開？
		HAL_UART_Abort_IT(&huart2);

		//割り込みの再開
		__enable_irq();
		HAL_NVIC_EnableIRQ(USART2_IRQn);

	 	HAL_UART_Receive_IT(&huart2, dma_rx_buff, 1);
	  }

	  grab_pin = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
	  link1_adc = g_ADCBuffer[0];
	  link2_adc = g_ADCBuffer[1];
	  link3_adc = g_ADCBuffer[2];
	  link4_adc = g_ADCBuffer[3];

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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 45;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 40000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 45;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 40000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 900;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 9000;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 500;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
