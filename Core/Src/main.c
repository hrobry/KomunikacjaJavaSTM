/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdbool.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

  // uint8_t communicationFrame  [13] = BIT STARTU= 0xFF , P , I , D , START , STOP , buttonLeft , buttonRight  , manual ,encoderUp,encoderDown,PWM, BIT STOPU = 0x00

 uint8_t P ;
 uint8_t  I ;
 uint8_t D ;
 uint8_t START ;
 uint8_t STOP ;
 uint8_t  leftButton;
 uint8_t rightButton ;
 uint8_t manual ;
 uint8_t  encoderUp;
 uint8_t encoderDown;
 uint8_t PWM;

   uint8_t  communicationFrame [13];
   void frameToName(){
P = communicationFrame [1];
I=communicationFrame [2];
D=communicationFrame [3];
START = communicationFrame [4];
STOP= communicationFrame [5];
leftButton= communicationFrame [6];
rightButton= communicationFrame [7];
manual= communicationFrame [8];
        communicationFrame [9]=encoderUp;
		communicationFrame [10]=encoderDown;
		communicationFrame [11]=PWM;
   }

double calculatePID(double setpoint, double pv) {

				double dt = 0.5;

							double Kp = 0.1;
							 double Ki = 0.1;
							double Kd = 0.1;

							double pre_error = 0;

							double integral = 0;
			    double error = setpoint - pv;

			    //P
			    double Pout = Kp * error;

			    //I
			    integral = integral + (error * dt);
			    double Iout = Ki * integral;

			    //D
			    double derivative = (error - pre_error) / dt;
			    double Dout = Kd * derivative;

			    //PID
			    double out = Pout + Iout + Dout;

			    pre_error = error;
			    return out;
			}


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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	static uint8_t Data[13]; // Tablica przechowujaca wysylana wiadomosc.

	////
	//HAL_UART_Transmit_DMA(&huart1, Data, 40); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
	HAL_UART_Receive_DMA(&huart2, communicationFrame, 13); // Ponowne włączenie nasłuchiwania
	//HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);


	frameToName();
	sprintf(Data, "%s", communicationFrame);
	HAL_UART_Transmit_DMA(&huart2, Data, 13);

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  uint16_t encoderCounterUp=0;
  uint16_t  encoderCounterDown=0;
  bool stateFirst=false;
  bool stateSecond=false;
  bool stateThird=false;
  bool stateFourth=false;
  bool stateFifth =false;
  char messageSend[64];
  char messageGet[64];
  char* mesage = 'recieved_message';
  double punktZero=0.0 ,PWM;
  bool left = false;
  bool right = false;
 int way;
  bool RecievedMSG = false;



  HAL_UART_Receive_DMA(&huart2, communicationFrame, 13);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // enkoder dol
  	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // enkoder gora
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // wysterowanie silnika
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  while (1)
  {
	encoderCounterUp = __HAL_TIM_GET_COUNTER(&htim4);
		encoderCounterDown = __HAL_TIM_GET_COUNTER(&htim3);
		if (START == 1&& STOP == 0 && manual == 0)
		{


			PWM =calculatePID(0,encoderCounterUp);
			 if(PWM>0)
						            {

						             HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,  GPIO_PIN_SET);
						             HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,  GPIO_PIN_RESET);
						             HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  GPIO_PIN_SET);
						             HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,  GPIO_PIN_RESET);
						             HAL_Delay(10);
						             __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM);
						            }else{


						            	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,  GPIO_PIN_RESET);
						            	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,  GPIO_PIN_SET);
						            	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  GPIO_PIN_RESET);
						            	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,  GPIO_PIN_SET);
						            	HAL_Delay(10);
						            	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM);
						            }
		}
		if (START == 0 && STOP == 1 && manual == 0)
		{

		}
		if (manual == 1 && leftButton==1 && rightButton == 0)
				{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,  GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,  GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,  GPIO_PIN_SET);
					HAL_Delay(10);
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 50);
				}
		if (manual == 1 && leftButton==1 && rightButton == 0)
						{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,  GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,  GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,  GPIO_PIN_SET);
					HAL_Delay(10);
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 50);
						}


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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

