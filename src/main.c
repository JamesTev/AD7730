
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
  * Author: James Teversham
  * Date: August/September 2018
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "serial_terminal.h"
#include "ad7730.h"
#include "dma_circular.h"
#include "opto_force.h"

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;

/* Private variables ---------------------------------------------------------*/
__IO ITStatus UartTxReady = SET;
__IO ITStatus SampleFlag = SET;

uint8_t ulReceivedValue[COMMS_TX_BUFFER_SIZE];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);

static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
static void Init_Gauges(void);
static void Prepare_Data(void);
static void Extract_Gauge_Data(volatile uint8_t * source_buffer, uint8_t * dest_buffer, uint8_t offset);
static void Read_Switch_Matrix(void);
static void update_position(uint8_t * switch_arr);

static volatile AD7730 gauge1;
static volatile AD7730 gauge2;
static volatile AD7730 gauge3;
static volatile AD7730 gauge4;

static volatile uint8_t switch_matrix[5];

uint8_t x = 0;
uint8_t y = 0; //triggered switch co-ords

uint16_t gauge_fixes[4];

uint32_t sample_counter = 0;
uint32_t num_readings = 0;

uint32_t bad_readings = 0;
uint32_t readings_caught = 0;
uint8_t new_reading = 1;

uint16_t start_readings = 0;
uint16_t end_readings = 0;

uint32_t attempts = 0;

uint32_t tx_count = 0;
uint32_t time_ms = 0;
uint32_t count = 0;
int8_t dir = 1;

uint8_t OF_error_flag = 0;


int main(void)
{
  //***************** System and Peripheral Init Functions ******************

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_SPI1_Init();

  //***************** Application Init Functions ******************
  time_ms = 0;
  Init_Gauges();

  Check_Calibration_Params(gauge1);
  Check_Calibration_Params(gauge2);
  Check_Calibration_Params(gauge3);
  Check_Calibration_Params(gauge4);

  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, 0); //check gain and offset values in gauge data buffers at this point

  MX_USART3_UART_Init();

  Config_OF(); //need to start continuous OF readings straight after
  Config_Idle_IRQ();
  while(!OF_Config_Complete); //wait until config has successfully complete
  HAL_UART_Receive_DMA(&huart3, DMA_RX_Buffer, 16); //** start continuous readings


  //************** Keep /sync and /reset pins high in normal operation

    AD7730_Start_Cont_Read(gauge1); //for continuous readings
    AD7730_Start_Cont_Read(gauge2); //for continuous readings
    AD7730_Start_Cont_Read(gauge3); //for continuous readings
    AD7730_Start_Cont_Read(gauge4); //for continuous readings

   time_ms=0;
   num_readings=0;
   HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, 0); //check gain and offset values in gauge data buffers at this point

  while (1)
  {
	  Read_Switch_Matrix(); //update values in switch matrix array
	  if(SampleFlag && new_reading){
		  //************** add && new_reading clause when using OF
		  readings_caught++;
		  new_reading = 0;

		  AD7730_Read_Cont(gauge1, (uint8_t *)&gauge1_data_buffer);
		  AD7730_Read_Cont(gauge2, (uint8_t *)&gauge2_data_buffer);
		  AD7730_Read_Cont(gauge4, (uint8_t *)&gauge4_data_buffer);
		  AD7730_Read_Cont(gauge3, (uint8_t *)&gauge3_data_buffer);

		  Prepare_Data();
		  if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)ulReceivedValue, COMMS_TX_BUFFER_SIZE)!= HAL_OK) //DMA UART transfer
		  {
			  _Error_Handler(__FILE__, __LINE__);
		  }

		  count++;
		  SampleFlag = 0;
	  }
	  else if(count >= 500){ //flash LD6 every 500 samples to show sampling is working

		  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, 1);
		  count=0;
	  }

  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if(huart->Instance==USART3){
		/** OptoForce Data Rx Callback **/
		num_readings++;
		new_reading = 1;
		if(num_readings == 1 && Validate_OF_Data_Packet((uint8_t *)&ack_buffer, OF_CONFIG_PACKET)!=OF_PACKET_VALID){
			HAL_UART_DMAStop(&huart3);
			attempts++;
			Config_OF(); //need to try configure again if initial config fails
			//__asm("BKPT");
			num_readings = 0;
			return;
		}

		if(num_readings > 1 && Validate_OF_Data_Packet((uint8_t *)&DMA_RX_Buffer, OF_DATA_PACKET)!=OF_PACKET_VALID){
			bad_readings++;
			OF_error_flag = 1;
			return;
		}

		if(num_readings ==1){
			start_readings = (DMA_RX_Buffer[4] <<8) | DMA_RX_Buffer[5]; //extract start readings from first OF packet
		}
	}
	else if(huart->Instance==USART2){
		/** SD Card Data Tx Callback **/
		UartTxReady = SET;
		tx_count++;
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* Turn LED3 on: Transfer error in reception/transmission process */
	__asm__("BKPT"); //software breakpoint
	if(huart->ErrorCode == HAL_UART_ERROR_ORE){ //overrun error
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);

	}
}

void HAL_SYSTICK_Callback(void){
	sample_counter++;
	//********** SAMPLE COUNTER CHANGED
	if(sample_counter==1){ //set for 1kHz
		SampleFlag = SET;
		sample_counter = 0;
	}

	if(readings_caught > 0){
		time_ms++;
	}
}

static void Read_Switch_Matrix(void){
	for(uint8_t i=0; i<5; i++){
		spi_tx_buffer[0] = 0b1 << i;
		Transmit_SR(); //set 1 row high at a time
		Receive_SR();
		switch_matrix[i] = spi_rx_buffer[0] & 0xFC; //only read 6 MSB from rx buffer (6 columns connected)
	}
	update_position(&switch_matrix);
}

static void Prepare_Data(void){

	uint8_t tempData[28];

	union{
		uint32_t i[6];
		uint8_t j[24]; //will be used to split 4x 32 bit numbers into 16 consecutive bytes
	}tempUnion;

	Extract_Gauge_Data(gauge1_data_buffer, tempData, 0);
	Extract_Gauge_Data(gauge2_data_buffer, tempData, 3);
	Extract_Gauge_Data(gauge3_data_buffer, tempData, 6);
	Extract_Gauge_Data(gauge4_data_buffer, tempData, 9); //TODO: improve this method

	/*
	 * Place Fx, Fy and Fz half-words from Optoforce straight after 12 AD7730 bytes.
	 * These 6 bytes are at byte positions 8-13 in OF packet
	 */
	for(uint8_t i = 0; i < 6; i++){
		tempData[12+i] = DMA_RX_Buffer[8+i];
	}

	/*
	 * Get x and y pos from switch array. Setting to ASCII 36 ($ sign) for
	 * testing
	 */
	uint8_t pos_x = x; //get the most recent x and y co-ords
	uint8_t pos_y = y;
	tempData[18] = pos_x;
	tempData[19] = pos_y;

	/*
	 * Transmit reading number (2 bytes) from OF
	 */
	tempData[20] = DMA_RX_Buffer[4];
	tempData[21] = DMA_RX_Buffer[5];

	uint8_t error_byte = 0;
	if(OF_error_flag){
		error_byte |= OF_BAD_PACKET; //set OF packet error flag in error byte
	}

	tempData[22] = error_byte;
	tempData[23] = 0; //dummy bytes to pad data packet to 24 bytes for CRC calculation

	uint8_t len = sizeof(tempUnion.j);

	//copy bytes to union
	for(uint8_t i=0; i < len; i++){
		tempUnion.j[len-i-1] = tempData[i];
	}
	reverseArray(tempUnion.i, 2);

	/*
	 * Generate 32 bit CRC using hardware CRC generator. Exclude start char (~) from CRC source so start from second element
	 */
	 uint32_t crcCalculated = HAL_CRC_Calculate(&hcrc, (uint32_t *)&tempUnion.i, 6); //generator poly 0x4C11DB7
	 for(uint8_t i=0; i< 4; i++){
	     tempData[24+i] = (crcCalculated >> (8*(3-i)))&0xFF; //unpack 4 bytes from CRC word and store in tempData array (MSB first)
	 }

	CommsTask_TransmitPacketStruct data=serialTerminal_packetize(tempData,sizeof(tempData)); //packetize into packet of bytes
	int i=0;
	for(i=data.bytes_to_tx;i<COMMS_TX_BUFFER_SIZE;i++)
	{
		data.data[i]=35; //pad remainder of COMMS_TX_BUFFER with zeros/#
	}
	for(i=0;i<COMMS_TX_BUFFER_SIZE;i++)
	{
		//ulReceivedValue[i+2]=data.data[i]; //populate DMA memory source buffer with packetized data (leave first 2 bytes for opcode)
		ulReceivedValue[i]=data.data[i];
	}

	OF_error_flag = 0; //reset an wait for next error flag
}

static void Extract_Gauge_Data(volatile uint8_t * source_buffer, uint8_t * dest_buffer, uint8_t offset){
	for (uint8_t i = 0; i < 3; i++) {
		dest_buffer[i+offset] = source_buffer[i];
	}
}

void Transmit_SR(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&spi_tx_buffer, 1, 1000); //will this just discard data shifted into RX data register?
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1); //rising edge to latch data into output register
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
}

void Receive_SR(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1); //rising edge to latch data into output register
	HAL_SPI_Receive(&hspi1, (uint8_t *)&spi_rx_buffer, 1, 1000); //will this just discard data shifted into RX data register?
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
}

void update_position(uint8_t * switch_arr){

  uint8_t row;
  uint8_t col;

  for(int i=0; i < 5; i++){
    if(switch_arr[i] > 0){
      row = i+1;
      break;
    }
  }
  for(int i=0; i < 5; i++){
    if((row << i) & 0x80 != 0){
      col = i+1;
    }
  }
  x = col - 3; //so column 3 is the x=0 line (y axis)
  y = row + 1; //still need to add logic for y
  if(row == 5 && col <= 3){ //adjustments for 5th (offset rows 5.1 and 5.2)
    y = 3;
    x = col - 2;
  }
  else if(row ==5 && col > 3){
    y = -3;
    x = col - 5;
  }
 y = row + 1; //still need to add logic for y
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

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; //TODO: investigate effect of changing this to 16 or less
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


/* USART2 init function */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 500000; //this is changed manually to 500 000 in hal_uart.c
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 1000000; //this is changed manually to 1Mbps in hal_uart.c
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
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
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0); //USART3 RX
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0); //USART2 TX
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/* CRC init function */
static void MX_CRC_Init(void)
{
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
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

  /*Configure GPIO pin : RCLK_Pin PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : load/shift tx pin PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  //Uncomment this section to control RESET and SYNC pins with software

//  GPIO_InitStruct.Pin = AD7730_RESET_PIN;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(AD7730_RESET_PORT, &GPIO_InitStruct);
//
//  GPIO_InitStruct.Pin = AD7730_SYNC_PIN;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(AD7730_SYNC_PORT, &GPIO_InitStruct);

}

/**
  * @brief Initialize slave select and ready pins for gauges
  * @retval None
  */
static void Init_Gauges(void){
	//HAL_GPIO_WritePin(AD7730_RESET_PORT, AD7730_RESET_PIN, 1); //hold /reset high in normal operation
	//HAL_GPIO_WritePin(AD7730_SYNC_PORT, AD7730_SYNC_PIN, 1); //hold /sync high in normal operation

	gauge1.SS_GPIO_Port = GAUGE1_SS_PORT;
	gauge1.RDY_GPIO_Port = GAUGE1_RDY_PORT;
	gauge1.SS_Pin = GAUGE1_SS_PIN;
	gauge1.RDY_Pin = GAUGE1_RDY_PIN;
	gauge1.data_buffer = (uint8_t *)&gauge1_data_buffer;
	gauge1.offset_buffer = (uint8_t *)&gauge1_offsets;
	AD7730_Init(gauge1);

	gauge2.SS_GPIO_Port = GAUGE2_SS_PORT;
	gauge2.RDY_GPIO_Port = GAUGE2_RDY_PORT;
	gauge2.SS_Pin = GAUGE2_SS_PIN;
	gauge2.RDY_Pin = GAUGE2_RDY_PIN;
	gauge2.data_buffer = (uint8_t *)&gauge2_data_buffer;
	gauge2.offset_buffer = (uint8_t *)&gauge2_offsets;
	AD7730_Init(gauge2);

	gauge3.SS_GPIO_Port = GAUGE3_SS_PORT;
	gauge3.RDY_GPIO_Port = GAUGE3_RDY_PORT;
	gauge3.SS_Pin = GAUGE3_SS_PIN;
	gauge3.RDY_Pin = GAUGE3_RDY_PIN;
	gauge3.data_buffer = (uint8_t *)&gauge3_data_buffer;
	gauge3.offset_buffer = (uint8_t *)&gauge3_offsets;
	AD7730_Init(gauge3);

	gauge4.SS_GPIO_Port = GAUGE4_SS_PORT;
	gauge4.RDY_GPIO_Port = GAUGE4_RDY_PORT;
	gauge4.SS_Pin = GAUGE4_SS_PIN;
	gauge4.RDY_Pin = GAUGE4_RDY_PIN;
	gauge4.data_buffer = (uint8_t *)&gauge4_data_buffer;
	gauge4.offset_buffer = (uint8_t *)&gauge4_offsets;
	AD7730_Init(gauge4);

    //software reset on begin
	AD7730_soft_reset(gauge1);
	AD7730_soft_reset(gauge2);
	AD7730_soft_reset(gauge3);
	AD7730_soft_reset(gauge4);

  	HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, 0);

	AD7730_Config(gauge1); //config gauge with all pre-configured settings
	AD7730_Config(gauge2); //config gauge with all pre-configured settings
	AD7730_Config(gauge3); //config gauge with all pre-configured settings
	AD7730_Config(gauge4); //config gauge with all pre-configured settings

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  __asm("BKPT");
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
