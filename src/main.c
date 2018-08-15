
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
#include "stm32f4xx_hal.h"
#include "serial_terminal.h"
#include "ad7730.h"


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
__IO ITStatus UartReady = RESET;
//uint8_t ulReceivedValue[COMMS_TX_BUFFER_SIZE+2]; //if including opcode for first 2 bytes
uint8_t ulReceivedValue[COMMS_TX_BUFFER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void Init_Gauges(void);
static void MX_DMA_Init(void);

static volatile AD7730 gauge1;
static volatile AD7730 gauge2;
static volatile AD7730 gauge3;
static volatile AD7730 gauge4;

int main(void)
{

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_DMA_Init();
  Init_Gauges();

  uint32_t count = 0;
  int8_t dir = 1;
  HAL_Delay(200);
  //AD7730_Start_Cont_Read(gauge1); //for continuous readings

  //************** Keep /sync and /reset pins high in normal operation

  while (1)
  {
	  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, 0);
	  while(HAL_GPIO_ReadPin(gauge2.RDY_GPIO_Port, gauge2.RDY_Pin) != GPIO_PIN_RESET); //wait for ready pin to go low
	  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, 1);
	  AD7730_Read(gauge2);

//	  AD7730_Read_Cont(gauge1);
//
//	  if(dir == 1){
//		  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, 0);
//	  }
//	  else{
//		  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, 1);
//	  }
//
//	  if(count > 1000){
//		  count=0;
//		  dir = dir*-1;
//	  }
//	  count++;
  }
  /* USER CODE END 3 */
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}

void AD7730_Reset(){
	HAL_GPIO_WritePin(AD7730_RESET_PORT, AD7730_RESET_PIN, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(AD7730_RESET_PORT, AD7730_RESET_PIN, GPIO_PIN_SET);
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE; //****This is for AD7730
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; //TODO: investigate effect of changing this
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0); //USART2 TX?
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE(); /************ Need GPIOE clock too!

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = AD7730_RESET_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AD7730_RESET_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = AD7730_SYNC_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AD7730_SYNC_PORT, &GPIO_InitStruct);

//  // ************ won't actually need these below
//
//  /*Configure GPIO pin : RDY_Pin */
//   GPIO_InitStruct.Pin = GPIO_PIN_4;
//   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//   /*Configure GPIO pin : SS_Pin */
//   GPIO_InitStruct.Pin = GPIO_PIN_1;
//   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/**
  * @brief Initialize slave select and ready pins for gauges
  * @retval None
  */
static void Init_Gauges(void){
	HAL_GPIO_WritePin(AD7730_RESET_PORT, AD7730_RESET_PIN, 1); //hold /reset high in normal operation
	//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET); //hold /reset high in normal operation
	HAL_GPIO_WritePin(AD7730_SYNC_PORT, AD7730_SYNC_PIN, 1); //hold /sync high in normal operation

//	gauge1.SS_GPIO_Port = GAUGE1_SS_PORT;
//	gauge1.RDY_GPIO_Port = GAUGE1_RDY_PORT;
//	gauge1.SS_Pin = GAUGE1_SS_PIN;
//	gauge1.RDY_Pin = GAUGE1_RDY_PIN;
//	AD7730_Init(gauge1);

	gauge1.SS_GPIO_Port = GPIOE;
	gauge1.RDY_GPIO_Port = GPIOE;
	gauge1.SS_Pin = GPIO_PIN_9;
	gauge1.RDY_Pin = GPIO_PIN_7;
	AD7730_Init(gauge1);

	gauge2.SS_GPIO_Port = GAUGE2_SS_PORT;
	gauge2.RDY_GPIO_Port = GAUGE2_RDY_PORT;
	gauge2.SS_Pin = GAUGE2_SS_PIN;
	gauge2.RDY_Pin = GAUGE2_RDY_PIN;
	AD7730_Init(gauge2);

	gauge3.SS_GPIO_Port = GAUGE3_SS_PORT;
	gauge3.RDY_GPIO_Port = GAUGE3_RDY_PORT;
	gauge3.SS_Pin = GAUGE3_SS_PIN;
	gauge3.RDY_Pin = GAUGE3_RDY_PIN;
	AD7730_Init(gauge3);

	gauge4.SS_GPIO_Port = GAUGE4_SS_PORT;
	gauge4.RDY_GPIO_Port = GAUGE4_RDY_PORT;
	gauge4.SS_Pin = GAUGE4_SS_PIN;
	gauge4.RDY_Pin = GAUGE4_RDY_PIN;
	AD7730_Init(gauge4);

	AD7730_Reset();

	AD7730_Config(gauge1); //config gauge with all pre-configured settings
	AD7730_Config(gauge2); //config gauge with all pre-configured settings
	AD7730_Config(gauge3); //config gauge with all pre-configured settings
	AD7730_Config(gauge4); //config gauge with all pre-configured settings

}


/* USER CODE BEGIN 4 */

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

/*****END OF FILE****/
