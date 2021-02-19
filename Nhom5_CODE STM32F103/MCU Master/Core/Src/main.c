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
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//===================CAN=======================
#define TCR_ID 0x1D
#define ID_MAIN 0x2C
#define MOTOR1_ID 0x3A
#define MOTOR2_ID 0x4B
CAN_FilterTypeDef hcanfilter;
uint16_t _FilterIdHigh=0xFFFF;
uint16_t _IdHigh=ID_MAIN;
CAN_RxHeaderTypeDef RxMessage; //CAN Bus Receive Header
CAN_TxHeaderTypeDef TxMessage;

//==============DATA=====================
uint8_t TX_Message[8] = {0,0,0,0,0,0,0,0};
uint8_t RX_Message[8] = {0,0,0,0,0,0,0,0};
uint8_t VL_Message[8] = {0,0,0,0,0,0,0,0};
uint8_t VR_Message[8] = {0,0,0,0,0,0,0,0};

//==============SENSOR===================
uint8_t positon_arr[4]={0};
float position=0;


//============-PID=======================
float Ki=0.0,Kp=0.0,Kd=0.0,Tsample=0.0;
float pre_I_Part=0.0,I_part=0.0,D_part=0.0,pre_Error=0.0,Error=0.0;
float Output=0.0,PID=0.0;
float setpoint_positon=0;

float VL=0.0,VR=0.0;
float throttle= 100; //rpm
uint8_t TCR_status=0;


void PID_Init(float _Kp,float _Ki,float _Kd,float _Tsample)
{

	pre_I_Part=0;
	I_part=0;
	D_part=0;
	pre_Error=0;
	Error=0;
	Output=0;
	PID=0;
	Kd=_Kd;
	Ki=_Ki;
	Kp=_Kp;
	Tsample=_Tsample;

}
void PID_Compute(float set_point,float feedback)
{
    Error = (float)(1.0*set_point-1.0*feedback);
    I_part = pre_I_Part + Error * Tsample;
    D_part = 1.0*(Error - pre_Error)/Tsample;
    PID = Kp*Error + Ki*I_part + Kd*D_part;
    pre_Error = Error;
    pre_I_Part = I_part;
}
float Get_PID(void)
{
	return PID;
}

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

	if (RxMessage.StdId == ID_MAIN)
	{

		if ((RX_Message[0]=='L') && (RX_Message[1]=='S'))
		{

			// READ LINE POSITION
			  for (uint8_t i=0;i<4;i++)
			  {
				  positon_arr[i] =  RX_Message[i+2];

			  }
			  byte2float(&position , positon_arr);
			  TCR_status = RX_Message[6];
			// READ LINE POSITION
		}

	}

}
float Get_position_feedback()
{
	return position;
}


//=====edit=======
#define CROSSB11 73
#define CROSSB12 77
#define CROSSB13 89
#define CROSSB14 107
#define CROSSB15 42
#define CROSSB16 43

#define CROSSB21 108
#define CROSSB22 124
#define CROSSB23 46
#define CROSSB24 62
#define CROSSB25 120
#define CROSSB26 56
#define CROSSB27 112
#define CROSSB28 32
#define CROSSB29 96
#define CROSSB2A 16
#define CROSSB2B 30
#define CROSSB2C 80
#define CROSSB2D 24
#define CROSSB2E 72

#define CROSSB31 88
#define CROSSB32 104
#define CROSSB33 120
#define CROSSB34 56
#define CROSSB35 40
#define CROSSB36 72
#define CROSSB37 80

#define CROSSB41 13
#define CROSSB42 9
#define CROSSB43 15
#define CROSSB44 11
#define CROSSB45 12


#define CROSSB51 9
#define CROSSB52 13
#define CROSSB53 27
#define CROSSB54 11
#define CROSSB55 14
#define CROSSB56 15

#define CROSSB61 127
#define CROSSB62 63
#define CROSSB63 59
#define CROSSB64 15


#define TIMING1 10
#define TIMING2 10
#define TIMING3 10
#define TIMING4 10
#define TIMING5 10
#define TIMING6 10


uint16_t timing_ms=0;
uint8_t plan_count =0;

void control_motor(void)
{
	   PID_Compute(setpoint_positon, Get_position_feedback());
	   VR= throttle+Get_PID();
	   VL= throttle-Get_PID();
	//	VR= throttle;
	//	VL= throttle;
	   if (throttle == 0 )
	   {
		   VR = 0;
		   VL = 0;
	   }
	   else if (throttle >0)
	   {
		   if (VR > 250)
		   	   {
		   		   VR=250;
		   	   }
		   	   else if (VR < 0)
		   	   {
		   		   VR=0;
		   	   }

		   	   if (VL > 250)
		   	   {
		   		   VL=250;
		   	   }
		   	   else if (VL < 0)
		   	   {
		   		   VL=0;
		   	   }
	   } //end throttle >0

   	   VL_Message[0]='S';
   	   VL_Message[1]='T';
   	   VL_Message[2]=(uint8_t)VL;
   	   VL_Message[3]=0;
   	   VL_Message[4]=0;
   	   VL_Message[5]=0;
   	   VL_Message[6]=0;
   	   VL_Message[7]=0;
   	   CANbus_Transmit(MOTOR2_ID, 8,VL_Message );

   	   VR_Message[0]='S';
   	   VR_Message[1]='P';
   	   VR_Message[2]=(uint8_t)VR;
   	   VR_Message[3]=0;
   	   VR_Message[4]=0;
   	   VR_Message[5]=0;
   	   VR_Message[6]=0;
   	   VR_Message[7]=0;
   	   CANbus_Transmit(MOTOR1_ID, 8, VR_Message);


}
void freerun_motor(uint8_t vel_L,uint8_t vel_R)
{


	   VL_Message[0]='S';
	   VL_Message[1]='T';
	   VL_Message[2]=(uint8_t)vel_L;
	   VL_Message[3]=0;
	   VL_Message[4]=0;
	   VL_Message[5]=0;
	   VL_Message[6]=0;
	   VL_Message[7]=0;
	   CANbus_Transmit(MOTOR2_ID, 8,VL_Message);

	   VR_Message[0]='S';
	   VR_Message[1]='P';
	   VR_Message[2]=(uint8_t)vel_R;
	   VR_Message[3]=0;
	   VR_Message[4]=0;
	   VR_Message[5]=0;
	   VR_Message[6]=0;
	   VR_Message[7]=0;
	   CANbus_Transmit(MOTOR1_ID, 8, VR_Message);
}
//void path_timing(void)
//{
//	timing_ms++;
//	if  (timing_ms == TIMING1)
//	{
//		plan_count = 0;
//	}
//	else if (timing_ms == TIMING2)
//	{
//		plan_count = 1;
//	}
//	else if  (timing_ms == TIMING3)
//	{
//		plan_count = 2;
//	}
//	else if  (timing_ms == TIMING4)
//	{
//		plan_count = 3;
//	}
//	else if  (timing_ms == TIMING5)
//	{
//		plan_count = 4;
//	}
//	else if  (timing_ms == TIMING6)
//	{
//		plan_count = 5;
//	}

//}
uint8_t left_status=0;
uint8_t right_status=0;
uint8_t pre_right_status=0;
void path_planing(void)
{
	left_status=TCR_status>>6;
	right_status=(TCR_status<<7);
	right_status=right_status>>7;
	pre_right_status=(TCR_status<<6);
	pre_right_status=pre_right_status>>7;

		if  (  (((right_status == 1) && (left_status == 1)) || (right_status == 1) || (left_status == 1) )&& (plan_count == 0) )
		{

			freerun_motor(115, 100);
			HAL_Delay(1000);
			plan_count = 1;

		}

	else if  ((left_status ==  1) && (plan_count == 1) )
	{
				freerun_motor((uint8_t)VL+20, (uint8_t)VR-30);
				HAL_Delay(500);
				plan_count = 2;
	}
	else if ( (left_status ==  1) && (plan_count == 2))
	{
				freerun_motor(100,100);
				HAL_Delay(1500);
				plan_count = 3;
	}
	else if ( (right_status ==  1) && (plan_count == 3))
	{
				freerun_motor((uint8_t)VL-30, (uint8_t)VR+20);
				HAL_Delay(500);
				plan_count = 4;
	}
   else if ( (right_status ==  1)  && (plan_count == 4))
	{
				freerun_motor(100, 120);

				HAL_Delay(1500);
				plan_count = 5;
	}
	else if ( (( (right_status ==  1) && (left_status ==  1)) || (right_status ==  1) ||  (left_status ==  1))  && (plan_count == 5))
	{
				freerun_motor(0, 0);

				HAL_Delay(4000);
				HAL_TIM_Base_Stop_IT(&htim3);
				plan_count = 6;
	}


	else
	{
		control_motor();
	}



}

uint16_t countx=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if (htim->Instance ==TIM3)
	{
//		countx++;
//		if (countx>1000)
//		{
//			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); //Toggle LED
//			HAL_Delay(2000);
//			countx=0;
//		}
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); //Toggle LED

//==================TEST=====================
	  //  control_motor();

//===================PathPlaning=============
	    path_planing();
//==================FREE================
	  //  freerun_motor(100,100);
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
  MX_CAN_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


  HAL_TIM_Base_Start_IT(&htim3);
  PID_Init(29,0,0.05,0.001);
  plan_count=0;
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// CANbus_Transmit(ID_MAIN, 8, TX_Message);
	 //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	// HAL_Delay(1000);

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
