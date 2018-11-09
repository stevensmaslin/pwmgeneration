
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h> //For use of strlen and other string related functions
#include <stdio.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define BOAT_RUDDER_PWMCHANNEL TIM_CHANNEL_2
#define BOAT_THROTTLE_PWMCHANNEL TIM_CHANNEL_1
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void user_pwm_setvalue(uint16_t value, uint32_t channel);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
char receiveBuffer[2];
char welcomeMsg[] = "RoBoat/Debug/Steering \r\n";
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3,BOAT_RUDDER_PWMCHANNEL); //Enable PWM for steering
  HAL_TIM_PWM_Start(&htim3, BOAT_THROTTLE_PWMCHANNEL); //Enable PWM for throttle
  HAL_UART_Transmit_IT(&huart2, (uint8_t *)welcomeMsg, sizeof(welcomeMsg));
  user_pwm_setvalue(157, BOAT_RUDDER_PWMCHANNEL); //Default steering
  user_pwm_setvalue(151, BOAT_THROTTLE_PWMCHANNEL); //Default throttle
  HAL_UART_Receive_IT(&huart2, (uint8_t *)receiveBuffer, 2); //Enable the UART receive interrupt
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*
	   * Superloop: Just blink the onboard LED on and off to show that the main program is running this loop
	   * The UART interrupts should break this loop and once serviced should jump back
	   * The PWM peripherals should not need servicing from inside this loop
	   */
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); //Toggle the LED
	  HAL_Delay((uint16_t)500); //Delay 500 ms
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 480-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 200-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* Function to modulate the pulse width of a PWM channel with a fixed frequency
 * Takes parameter value which currently is the number of counts of the 100 kHz timer during which the pulse will be high
 * Calculate the duty cycle by d = value/1000
 * Takes parameter channel which is the PWM channel to modulate
 * */
void user_pwm_setvalue(uint16_t value, uint32_t channel)
{
	HAL_TIM_PWM_Stop(&htim3, channel);
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, channel);
	HAL_TIM_PWM_Start(&htim3, channel);
}

/*
 * UART Receive Complete Callback function
 * Invoked when HAL_UART_Receive_IT fills the buffer it is given
 * Currently takes a single byte command and one other character (suggest enter key) and sets throttle/steering control pwm
 * Commands not case sensitive
 * ==========================================
 * Command		Action
 * ------------------------------------------
 * R			Steer Rudder Full Right
 * L			Steer Rudder Full Left
 * C or S		Return Rudder to Center
 * F			Forward Throttle (low power)
 * V or B		Reverse Throttle (low power)
 * X			Kill Throttle
 * ==========================================
 *
 * Resets the UART RX Interrupt once complete so we can receive more data and interrupt the superloop when this happens
 * Buggy to call HAL_UART_Transmit_IT in this callback, use HAL_UART_Transmit (blocking mode transmission) instead
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//Set default values to send via PWM
	uint16_t degree = 157;
	uint32_t ch = BOAT_RUDDER_PWMCHANNEL;
	//Get the first character from the buffer as the command
	char cmd = receiveBuffer[0];
	//TODO Rewrite this as a switch statement
	if(cmd == 'r' || cmd == 'R')
	{
		degree = 101;
		ch = BOAT_RUDDER_PWMCHANNEL;
		char rMsg[] = "Steering Right\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)rMsg, sizeof(rMsg), 0xFFFF);
	}
	else if(cmd == 'l' || cmd == 'L')
	{
		ch = BOAT_RUDDER_PWMCHANNEL;
		degree = 200;
		char lMsg[] = "Steering Left\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)lMsg, sizeof(lMsg), 0xFFFF);
	}
	else if(cmd == 'c' || cmd == 'C' || cmd == 's' || cmd == 'S')
	{
		ch = BOAT_RUDDER_PWMCHANNEL;
		degree = 157;
		char cMsg[] = "Steering Straight\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)cMsg, sizeof(cMsg), 0xFFFF);
	}
	else if(cmd == 'f' || cmd == 'F')
	{
		ch = BOAT_THROTTLE_PWMCHANNEL; //Select throttle timer output channel
		degree = 160;
		char fMsg[] = "Throttle forward 16.0%\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)fMsg, sizeof(fMsg), 0xFFFF);
	}
	else if(cmd == 'x' || cmd == 'X')
	{
		ch = BOAT_THROTTLE_PWMCHANNEL;
		degree = 150;
		char xMsg[] = "Throttle off 15.0%\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)xMsg, sizeof(xMsg), 0xFFFF);
	}
	else if(cmd == 'v' || cmd == 'V' || cmd == 'b' || cmd == 'B')
	{
		ch = BOAT_THROTTLE_PWMCHANNEL;
		degree = 135;
		char vMsg[] = "Throttle reverse 13.5%\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)vMsg, sizeof(vMsg), 0xFFFF);
	}
	user_pwm_setvalue(degree, ch); //Modify the output PWM as commanded
	HAL_UART_Receive_IT(&huart2, (uint8_t *)receiveBuffer, 2); //Reset the receive interrupt to regenerate this callback function when more data received
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
