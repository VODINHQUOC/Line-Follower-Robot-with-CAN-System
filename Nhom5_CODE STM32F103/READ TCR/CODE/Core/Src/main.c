/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
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

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void float2byte(float float_input,uint8_t* data_in)
{
	union class1{
		float value_float;
		uint8_t data[4];
	};
	union class1 t;
	t.value_float=float_input;
	data_in[0]=t.data[0];
	data_in[1]=t.data[1];
	data_in[2]=t.data[2];
	data_in[3]=t.data[3];

}
void byte2float(float *floatout,uint8_t* datain)
{
	union class2{
		float value_float;
		uint8_t data[4];
	};
  union class2 t;
	t.data[0]=datain[0];
	t.data[1]=datain[1];
	t.data[2]=datain[2];
	t.data[3]=datain[3];
	*floatout=t.value_float;

}
void short2byte(uint16_t short_input,uint8_t* datain)
{
	union qshort2byte
	{
		uint16_t short_value;
		uint8_t data[2];
	};
	union qshort2byte tar;
	tar.short_value=short_input;
	datain[0]=tar.data[0];
	datain[1]=tar.data[1];
}
void byte2short(uint16_t* short_output,uint8_t* datain)
{
	union qbyte2short
	{
		uint16_t short_value;
		uint8_t data[2];

	};
	union qbyte2short tar;
	tar.data[0]=datain[0];
	tar.data[1]=datain[1];
	*short_output= tar.short_value;

}

#define TCR_ID 0x1D
#define ID_MAIN 0x2C
#define MOTOR1_ID 0x3A
#define MOTOR2_ID 0x4B
CAN_FilterTypeDef hcanfilter;
uint16_t _FilterIdHigh=0xFFFF;
uint16_t _IdHigh=TCR_ID;
CAN_RxHeaderTypeDef RxMessage; //CAN Bus Receive Header
CAN_TxHeaderTypeDef TxMessage;
uint8_t TX_Message[8] = {0,0,0,0,0,0,0,0};
uint8_t RX_Message[8] = {0,0,0,0,0,0,0,0};


void CANbus_Transmit(uint32_t StdID, uint8_t can_DLC,uint8_t* can_Data)
{
	uint32_t TransmitMailbox;
	TxMessage.StdId=StdID;
	TxMessage.ExtId=0;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.DLC=can_DLC;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.TransmitGlobalTime = DISABLE;
	HAL_CAN_AddTxMessage(&hcan, &TxMessage, can_Data, &TransmitMailbox);
	uint32_t k=0;
	while((HAL_CAN_IsTxMessagePending(&hcan, TransmitMailbox) != 1) && (k!=0xFFFF))
	{
		k++;

	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxMessage, RX_Message); //Receive CAN bus message to canRX buffer

	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	if (RxMessage.StdId == TCR_ID)
	{

		if((RX_Message[0] == 'R') && (RX_Message[0] == 'S'))
		{
			HAL_TIM_Base_Start_IT(&htim2);
		}
		else if ((RX_Message[0] == 'S') && (RX_Message[0] == 'T'))
		{
			HAL_TIM_Base_Stop_IT(&htim2);
		}
		// Start TIMER2


	}

}


float position_final=0;

uint8_t positon_arr[4]={0};
float test_number =0;
float TCR1_xpos=-3,TCR2_xpos=-2,TCR3_xpos=-1,TCR4_xpos=0,TCR5_xpos=1,TCR6_xpos=2,TCR7_xpos=3;
uint16_t DATA_TCR[7]={0,0,0,0,0,0,0};

uint8_t TCR1_Status=0;
uint8_t TCR2_Status=0;
uint8_t TCR3_Status=0;
uint8_t TCR4_Status=0;
uint8_t TCR5_Status=0;
uint8_t TCR6_Status=0;
uint8_t TCR7_Status=0;
uint8_t TCR_status=0;



uint16_t PRE_DATA_TCR[7]={0,0,0,0,0,0,0};
uint16_t TCR_min[7]={125,180,150,180,175,210,190};
uint16_t TCR_max[7]={3100,3160,2900,3280,3348,3280,3400};
uint16_t ymax=3000,ymin=100;
void calibrate_linesensor(void)
{
	for(uint8_t i=0;i<7;i++)
	{
		DATA_TCR[i]=(uint16_t)(ymin+1.0*(ymax-ymin)*(PRE_DATA_TCR[i]-TCR_min[i])/(TCR_max[i]-TCR_min[i])*1.0);
	}

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == TIM2)
	{
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		  TCR_status=0x00;

		 // HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&DATA_TCR, 7);
		  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&PRE_DATA_TCR, 7);
		  TCR1_Status=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
		  TCR2_Status=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
		  TCR3_Status=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
		  TCR4_Status=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
		  TCR5_Status=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
		  TCR6_Status=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
		  TCR7_Status=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);

		  calibrate_linesensor();

		  TCR_status= TCR_status|(TCR1_Status<<6);
		  TCR_status= TCR_status|(TCR2_Status<<5);
		  TCR_status= TCR_status|(TCR3_Status<<4);
		  TCR_status= TCR_status|(TCR4_Status<<3);
		  TCR_status= TCR_status|(TCR5_Status<<2);
		  TCR_status= TCR_status|(TCR6_Status<<1);
		  TCR_status= TCR_status|(TCR7_Status<<0);



		//  position_final = 3.14;
		  position_final =(float)((-3.0*DATA_TCR[0]-2.0*DATA_TCR[1]-1.0*DATA_TCR[2]+0.0*DATA_TCR[3]+1.0*DATA_TCR[4]+2.0*DATA_TCR[5]+3.0*DATA_TCR[6])/(1.0*DATA_TCR[0]+1.0*DATA_TCR[1]+1.0*DATA_TCR[2]+1.0*DATA_TCR[3]+1.0*DATA_TCR[4]+1.0*DATA_TCR[5]+1.0*DATA_TCR[6]));
		  float2byte(position_final, positon_arr);
		 // byte2float(&test_number, positon_arr);

		  TX_Message[0] = 'L';
		  TX_Message[1] = 'S';
		  TX_Message[2] = positon_arr[0];
		  TX_Message[3] = positon_arr[1];
		  TX_Message[4] = positon_arr[2];
		  TX_Message[5] = positon_arr[3];
		  TX_Message[6] = TCR_status;
		  TX_Message[7] = 0x00;


		  CANbus_Transmit(ID_MAIN,8,TX_Message);


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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Stop(&htim2);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  //HAL_Delay(500);
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 7;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  hcanfilter.FilterActivation=CAN_FILTER_ENABLE;
  hcanfilter.FilterMode=CAN_FILTERMODE_IDMASK;
  hcanfilter.FilterScale=CAN_FILTERSCALE_16BIT;
  hcanfilter.FilterMaskIdHigh=(_FilterIdHigh << 5);
  hcanfilter.FilterMaskIdLow=0x0000;

  hcanfilter.FilterBank=2;
  hcanfilter.FilterIdHigh=_IdHigh;
  hcanfilter.FilterIdLow=0x0000;
  hcanfilter.FilterFIFOAssignment=CAN_FILTER_FIFO1;
  hcanfilter.SlaveStartFilterBank=14;

  HAL_CAN_ConfigFilter(&hcan,&hcanfilter); //Initialize CAN Filter
  HAL_CAN_Start(&hcan); //Initialize CAN Bus
  HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO1_MSG_PENDING);// Initialize
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA11
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
