/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "motor.h"
#include "odometry.h"
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

/* USER CODE BEGIN PV */
CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef TxMsg;
CAN_RxHeaderTypeDef RxMsg;

uint8_t CAN_Rx_Data[8];
uint32_t TxMailbox = 0;


Motor_TypeDef Roda[3]={
		{.MotorID=1},
		{.MotorID=3},
		{.MotorID=2}
};

uint32_t time_a,time_b;


short int encoder[2];

uint8_t gyroKirim;
uint8_t gyroTerima[8];

int16_t gyroBuff;
float gyroRAW;
float heading,offsetGyro;
uint8_t status_gyro;


char uart5_kirim[128],uart5_terima;
char uart1_kirim[12],uart1_terima;
char uart5_status;
char buff_PC[8];
char bitLamp;

int test_var;

int time_scale;


int speedLocal[3];

uint8_t statusControl;

int speedPC[3];
int time_lamp;

float pos_x,pos_y;

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM3_Init();
  MX_DMA_Init();
  MX_UART5_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);


  TxMsg.IDE = CAN_ID_STD;
  TxMsg.RTR = CAN_RTR_DATA;

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

  sFilterConfig.FilterIdHigh = 0X0000;
  sFilterConfig.FilterIdLow = 0X0000;

  sFilterConfig.FilterMaskIdHigh = 0X0000;
  sFilterConfig.FilterMaskIdLow = 0X0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 0;

  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
  HAL_CAN_Start(&hcan1);

  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  HAL_Delay(2000);

  HAL_UART_Receive_DMA(&huart2, gyroTerima, 8);


  gyroKirim = 0xA5;
  HAL_UART_Transmit(&huart2, &gyroKirim, 1, 1000);
  gyroKirim = 0x54;
  HAL_UART_Transmit(&huart2, &gyroKirim, 1, 1000);
  HAL_Delay(1000);

  gyroKirim = 0xA5;
  HAL_UART_Transmit(&huart2, &gyroKirim, 1, 1000);
  gyroKirim = 0x55;
  HAL_UART_Transmit(&huart2, &gyroKirim, 1, 1000);
  HAL_Delay(1000);


  gyroKirim = 0xA5;
  HAL_UART_Transmit(&huart2, &gyroKirim, 1, 1000);
  gyroKirim = 0x52;
  HAL_UART_Transmit(&huart2, &gyroKirim, 1, 1000);
  HAL_Delay(100);

  HAL_UART_Receive_DMA(&huart1, (uint8_t*)&uart1_terima, 1);
  HAL_UART_Receive_DMA(&huart5, (uint8_t*)&uart5_terima, 1);

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

  HAL_TIM_Base_Start_IT(&htim6);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



	  for(int i=0;i<1;i++)
	  {
		  MotorKontrolHandler(&Roda[i]);
		  HAL_Delay(1);
	  }
	  MotorReadVoltage(&Roda[0]);


	  HAL_Delay(1);



//	  encoder[0] = TIM3->CNT;
//	  encoder[1] = TIM8->CNT;


	  if(HAL_GetTick()-time_a>10)
	  {
		  time_a = HAL_GetTick();

/*
		  sprintf(uart1_kirim,"%d#1\n",test_var);
		  sprintf(uart5_kirim,"%d#5\n",test_var);
		  HAL_UART_Transmit(&huart1, (uint8_t*)uart1_kirim, strlen(uart1_kirim), 100);
		  HAL_UART_Transmit(&huart5, (uint8_t*)uart5_kirim, strlen(uart1_kirim), 100);

		  if(++test_var>100)
			  test_var = 0;
*/



		  sprintf(uart5_kirim,"%0.2f,%0.2f,%0.2f,%d\n",pos_x,
				  pos_y,heading,statusControl);
		  HAL_UART_Transmit(&huart5, (uint8_t*)uart5_kirim, strlen(uart5_kirim), 1000);

	  }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)  //??????
{

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMsg, CAN_Rx_Data);


	MotorRxDataHandler(&Roda[0], &RxMsg, CAN_Rx_Data);



}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		if(gyroTerima[0]==0xAA && gyroTerima[7]==0x55)
		{
			gyroBuff = ((int16_t)gyroTerima[1])<<8 | (int16_t)gyroTerima[2];
			gyroRAW  = (float)gyroBuff * 0.01;

			if(status_gyro==0)
			{
				offsetGyro = gyroRAW;
				status_gyro=1;
			}

			heading = gyroRAW - offsetGyro;

			if(heading>180)
				heading -= 360;
			else if(heading<-180)
				heading += 360;
		}
	}
	else if(huart->Instance == UART5)
	{
		if(uart5_terima=='m' && uart5_status == 0)
		{
			uart5_status = 1;
			HAL_UART_Receive_DMA(&huart5, (uint8_t*)&uart5_terima, 1);
		}
		else if(uart5_terima=='r' && uart5_status==1)
		{
			uart5_status = 2;
			HAL_UART_Receive_DMA(&huart5, (uint8_t*)&uart5_terima, 1);
		}
		else if(uart5_terima=='i' && uart5_status==2)
		{
			uart5_status =3;
			HAL_UART_Receive_DMA(&huart5,(uint8_t*)buff_PC,sizeof(buff_PC));
		}
		else if(uart5_status==3)
		{

			memcpy(&speedPC[0],buff_PC,2);
			memcpy(&speedPC[1],buff_PC+2,2);
			memcpy(&speedPC[2],buff_PC+4,2);
			bitLamp = buff_PC[6];
			statusControl = buff_PC[7];

			uart5_status = 0;
			HAL_UART_Receive_DMA(&huart5, (uint8_t*)&uart5_terima, 1);
		}
		else
		{
			uart5_status=0;
			HAL_UART_Receive_DMA(&huart5, (uint8_t*)&uart5_terima, 1);
		}

	}
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{



	if(htim->Instance == TIM6)
	{


		OdomCalc(&htim3, &htim8, heading);
		pos_x = odomGlobalPosition[0]*-1;
		pos_y = odomGlobalPosition[1]*-1;


		if(++time_scale>10)
		{
			time_scale = 0;

			motorLocalSpeed(speedLocal[0], speedLocal[1], speedLocal[2]);
//			Roda[0].TargetSpeed = motorSpeedSP[0];
			Roda[1].TargetSpeed = motorSpeedSP[1];
			Roda[2].TargetSpeed = motorSpeedSP[2];


			switch(statusControl)
			{
			case 0:

				if(uart1_terima=='a')
				{
					speedLocal[0] = 0;
					speedLocal[1] = 30;
					speedLocal[2] = 0;
				}
				else if(uart1_terima=='c')
				{
					speedLocal[0] = -30;
					speedLocal[1] = 0;
					speedLocal[2] = 0;
				}
				else if(uart1_terima=='d')
				{
					speedLocal[0] = 0;
					speedLocal[1] = -30;
					speedLocal[2] = 0;
				}
				else if(uart1_terima=='b')
				{
					speedLocal[0] = 30;
					speedLocal[1] = 0;
					speedLocal[2] = 0;
				}
				else if(uart1_terima=='C')
				{
					speedLocal[0] = 0;
					speedLocal[1] = 0;
					speedLocal[2] = 20;
				}
				else if(uart1_terima=='B')
				{
					speedLocal[0] = 0;
					speedLocal[1] = 0;
					speedLocal[2] = -20;
				}
				else
				{
					speedLocal[0]=0;
					speedLocal[1]=0;
					speedLocal[2]=0;
				}


				if(++time_lamp>50)
				{
					time_lamp = 0;
					HAL_GPIO_TogglePin(O0_GPIO_Port, O0_Pin);
					HAL_GPIO_TogglePin(O1_GPIO_Port, O1_Pin);
					HAL_GPIO_TogglePin(O2_GPIO_Port, O2_Pin);
				}


				break;

			case 1:

				speedLocal[0] = speedPC[0];
				speedLocal[1] = speedPC[1];
				speedLocal[2] = speedPC[2];

				HAL_GPIO_WritePin(O0_GPIO_Port, O0_Pin, bitLamp & 0x01);
				HAL_GPIO_WritePin(O1_GPIO_Port, O1_Pin, (bitLamp>>1) & 0x01);
				HAL_GPIO_WritePin(O2_GPIO_Port, O2_Pin, (bitLamp>>2) & 0x01);
				HAL_GPIO_WritePin(O3_GPIO_Port, O3_Pin, (bitLamp>>3) & 0x01);
				HAL_GPIO_WritePin(O4_GPIO_Port, O4_Pin, (bitLamp>>4) & 0x01);
				HAL_GPIO_WritePin(O5_GPIO_Port, O5_Pin, (bitLamp>>5) & 0x01);
				HAL_GPIO_WritePin(O6_GPIO_Port, O6_Pin, (bitLamp>>6) & 0x01);

				break;
			}

		}


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
