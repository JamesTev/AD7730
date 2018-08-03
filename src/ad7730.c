/*
 * ad7730.c
 *
 *  Created on: 02 Aug 2018
 *  Author: James Teversham
 */

#include "ad7730.h"
#include "stm32f4xx_hal.h"
#include "main.h"

extern SPI_HandleTypeDef hspi2;

__IO ITStatus CONT_READ_STARTED = RESET;

void AD7730_Init(AD7730 gauge)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/*Configure GPIO pin : SS_Pin */
	GPIO_InitStruct.Pin = gauge.SS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(gauge.SS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : RDY_Pin */
	GPIO_InitStruct.Pin = gauge.RDY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(gauge.RDY_GPIO_Port, &GPIO_InitStruct);

	AD7730_Config(gauge); //config gauge with all pre-configured settings
}

//Think by not declaring static, automatically extern
void AD7730_Config(AD7730 gauge){


	spi_tx_buffer[0] = CR_SINGLE_READ|CR_FILTER_REGISTER; //see if can just pass pointer to this as arg in function below
	Tx_AD7730(1, gauge); //will this just discard data shifted into RX data register?
	Rx_AD7730(3, gauge);

	//-------------- Filter Config -----------------------
	spi_tx_buffer[0] = CR_SINGLE_WRITE|CR_FILTER_REGISTER; //see if can just pass pointer to this as arg in function below
	Tx_AD7730(1, gauge); //will this just discard data shifted into RX data register?

	spi_tx_buffer[0] = FR2_SINC_AVERAGING_512;
	spi_tx_buffer[1] = FR1_SKIP_OFF|FR1_FAST_OFF;
	spi_tx_buffer[2] = FR0_CHOP_ON;
	Tx_AD7730(3, gauge); //will this just discard data shifted into RX data register?

	//check if settings were stored - read filter reg:
	spi_tx_buffer[0] = CR_SINGLE_READ | CR_FILTER_REGISTER; //see if can just pass pointer to this as arg in function below
	Tx_AD7730(1, gauge); //will this just discard data shifted into RX data register?
	Rx_AD7730(3, gauge);

	//-------------- DAC Config -----------------------
	spi_tx_buffer[0] = CR_SINGLE_WRITE | CR_DAC_REGISTER;
	Tx_AD7730(1, gauge); //will this just discard data shifted into RX data register?

	spi_tx_buffer[0] = DACR_OFFSET_SIGN_POSITIVE | DACR_OFFSET_NONE;
	Tx_AD7730(1, gauge);

	//-------------- Internal Full Scale Calibration (max load) -----------------------
	spi_tx_buffer[0] = CR_SINGLE_WRITE | CR_MODE_REGISTER;
	Tx_AD7730(1, gauge); //will this just discard data shifted into RX data register?

	spi_tx_buffer[0] = MR1_MODE_INTERNAL_FULL_CALIBRATION | CURRENT_MODE_1_SETTINGS; //***** TODO: CHECK IF CORRECT VOLTAGE RANGE FOR BIPOLAR 0-10mV?
	spi_tx_buffer[1] = CURRENT_MODE_0_SETTINGS;
	Tx_AD7730(2, gauge);

	while(HAL_GPIO_ReadPin(gauge.RDY_GPIO_Port, gauge.RDY_Pin) != GPIO_PIN_RESET); //wait for ready pin to go low after calibration

	//-------------- Internal Zero Calibration (min load) -----------------------
	spi_tx_buffer[0] = CR_SINGLE_WRITE | CR_MODE_REGISTER;
	Tx_AD7730(1, gauge); //will this just discard data shifted into RX data register?

	spi_tx_buffer[0] = MR1_MODE_INTERNAL_ZERO_CALIBRATION | CURRENT_MODE_1_SETTINGS;
	spi_tx_buffer[1] = CURRENT_MODE_0_SETTINGS;
	Tx_AD7730(2, gauge);

	while(HAL_GPIO_ReadPin(gauge.RDY_GPIO_Port, gauge.RDY_Pin) != GPIO_PIN_RESET); //wait for ready pin to go low after calibration
}

/*
 * Accepts number of bytes to transfer from spi_tx_buffer array to specified gauge (1 - 4)
 */
void Tx_AD7730(uint16_t length, AD7730 gauge){

	HAL_GPIO_WritePin(gauge.SS_GPIO_Port, gauge.SS_Pin, GPIO_PIN_RESET); //slave select low to begin
	HAL_SPI_Transmit(&hspi2, (uint8_t *)&spi_tx_buffer, length, 5000); //will this just discard data shifted into RX data register?
	HAL_GPIO_WritePin(gauge.SS_GPIO_Port, gauge.SS_Pin, GPIO_PIN_SET); //slave select low to begin
}

/*
 * Accepts number of bytes to read from spi_tx_buffer array to specified gauge (1 - 4)
 */
void Rx_AD7730(uint16_t length, AD7730 gauge){

	spi_tx_buffer[0] = READ_ONLY; //transfer 0xFF (set MSB) to prevent writing to CR accidentally during reading
	spi_tx_buffer[1] = READ_ONLY;
	spi_tx_buffer[2] = READ_ONLY;

	HAL_GPIO_WritePin(gauge.SS_GPIO_Port, gauge.SS_Pin, GPIO_PIN_RESET); //slave select low to begin
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)&spi_tx_buffer, (uint8_t *)&spi_rx_buffer, length, 5000); //will this just discard data shifted into RX data register?
	HAL_GPIO_WritePin(gauge.SS_GPIO_Port, gauge.SS_Pin, GPIO_PIN_SET); //slave select low to begin
}

void AD7730_Read(AD7730 gauge){

	// need to put AD7730 into single read mode before every read (returns to idle after every conversion)
	spi_tx_buffer[0] = CR_SINGLE_WRITE | CR_MODE_REGISTER; //write to comms register and set next write to mode reg
	Tx_AD7730(1, gauge); //0x02

	spi_tx_buffer[0] = MR1_MODE_SINGLE | CURRENT_MODE_1_SETTINGS; //write to mode reg startin cont readings for 0-10mV range TODO: check this for bipolar
	spi_tx_buffer[1] = CURRENT_MODE_0_SETTINGS;
	Tx_AD7730(2, gauge);

	spi_tx_buffer[0] = CR_SINGLE_READ | CR_DATA_REGISTER;
	Tx_AD7730(1, gauge);

	spi_tx_buffer[0] = READ_ONLY; //transfer 0xFF (set MSB) to prevent writing to CR accidentally during reading
	spi_tx_buffer[1] = READ_ONLY;
	spi_tx_buffer[2] = READ_ONLY;

	HAL_GPIO_WritePin(gauge.SS_GPIO_Port, gauge.SS_Pin, GPIO_PIN_RESET); //slave select low to begin
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)&spi_tx_buffer, (uint8_t *)&spi_rx_buffer, 3, 5000); //assume this will receive the bytes too?
	HAL_GPIO_WritePin(gauge.SS_GPIO_Port, gauge.SS_Pin, GPIO_PIN_SET); //slave select low to begin
}

/*
 * Config AD7730 for cont read mode. Note: should be prepared to take readings at specified rate straight after invocation of this fn
 */
void AD7730_Start_Cont_Read(AD7730 gauge){

	spi_tx_buffer[0] = CR_SINGLE_WRITE | CR_MODE_REGISTER; //write to comms register and set next write to mode reg
	Tx_AD7730(1, gauge); //0x02

	spi_tx_buffer[0] = MR1_MODE_CONTINUOUS | CURRENT_MODE_1_SETTINGS; //write to mode reg startin cont readings for 0-10mV range TODO: check this for bipolar
	spi_tx_buffer[1] = CURRENT_MODE_0_SETTINGS;
	Tx_AD7730(2, gauge); //0x2180

	spi_tx_buffer[0] = CR_CONTINUOUS_READ_START | CR_DATA_REGISTER;
	Tx_AD7730(1, gauge); //0x21

	while(HAL_GPIO_ReadPin(gauge.RDY_GPIO_Port, gauge.RDY_Pin) != GPIO_PIN_RESET); //wait for ready pin to go low when cont readings start

	CONT_READ_STARTED = SET; //TODO: will this work for all gauges? first gauge can set it but subsequent ones don't need to..
}

void AD7730_Read_Cont(AD7730 gauge){
	if(CONT_READ_STARTED==SET){ //prevent attempt at cont read before starting
		spi_tx_buffer[0] = 0;
		spi_tx_buffer[1] = 0;
		spi_tx_buffer[2] = 0; //need to transfer all 0 bytes to keep DIN of AD7730 low prevent device reset during cont read

		while(HAL_GPIO_ReadPin(gauge.RDY_GPIO_Port, gauge.RDY_Pin) != GPIO_PIN_RESET); //wait for ready pin to go low

		HAL_GPIO_WritePin(gauge.SS_GPIO_Port, gauge.SS_Pin, GPIO_PIN_RESET); //slave select low to begin
		HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)&spi_tx_buffer, (uint8_t *)&spi_rx_buffer,3, 5000); //assume this will receive the bytes too?
		HAL_GPIO_WritePin(gauge.SS_GPIO_Port, gauge.SS_Pin, GPIO_PIN_SET); //slave select low to begin

		// uint32_t result = x3 + (x2 << 8) + (x1 << 16); //consider returning int32_t using something like this (still need to deal with MSB for signed)
	}
}

/*
 * Stop cont read. Returns to idle state
 */
void AD7730_Stop_Cont_Read(AD7730 gauge){
	spi_tx_buffer[0] = CR_SINGLE_WRITE | CR_CONTINUOUS_READ_STOP; //0x30
	Tx_AD7730(1, gauge);
	CONT_READ_STARTED = RESET;
}

int32_t Process_Buffer(void){
	// Replicate the most significant bit to pad out a 32-bit signed integer
	uint8_t filler = 0x00;
	if (spi_rx_buffer[2] & 0x80) {
		filler = 0xFF;
	}

//	// Construct a 32-bit signed integer
//	value = ( static_cast<unsigned long>(filler) << 24
//			| static_cast<unsigned long>(data[2]) << 16
//			| static_cast<unsigned long>(data[1]) << 8
//			| static_cast<unsigned long>(data[0]) );
//
//	return static_cast<long>(value);
}



