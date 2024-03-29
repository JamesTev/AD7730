/*
 *  ad7730.c
 *  Created on: 02 Aug 2018
 *  Author: James Teversham (jamestevers@gmail.com)
 */

#include "ad7730.h"
#include "stm32f4xx_hal.h"
#include "main.h"

extern SPI_HandleTypeDef hspi2;
extern volatile uint8_t gauge1_data_buffer[3];
extern volatile uint8_t gauge2_data_buffer[3];
extern volatile uint8_t gauge3_data_buffer[3];
extern volatile uint8_t gauge4_data_buffer[3]; //do we need to define these in a source file?

extern volatile uint8_t gauge1_offset[3];
extern volatile uint8_t gauge2_offset[3];
extern volatile uint8_t gauge3_offset[3];
extern volatile uint8_t gauge4_offset[3];

extern char initial_config_done=0;

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

	HAL_GPIO_WritePin(gauge.SS_GPIO_Port, gauge.SS_Pin, GPIO_PIN_SET); //set /SS high at first
}

//Think by not declaring static, automatically extern
void AD7730_Config(AD7730 gauge){


	spi_tx_buffer[0] = CR_SINGLE_READ|CR_FILTER_REGISTER; //see if can just pass pointer to this as arg in function below
	Tx_AD7730(1, gauge); //will this just discard data shifted into RX data register?
	Rx_AD7730(3, gauge);

	//-------------- Filter Config -----------------------
	spi_tx_buffer[0] = CR_SINGLE_WRITE|CR_FILTER_REGISTER; //see if can just pass pointer to this as arg in function below
	Tx_AD7730(1, gauge); //will this just discard data shifted into RX data register?

	//************** Initially perform max averaging of sinc filter for calibration

	spi_tx_buffer[0] = FR2_SINC_AVERAGING_2048;

	spi_tx_buffer[1] = FR1_SKIP_OFF|FR1_FAST_OFF;
	spi_tx_buffer[2] = FR0_CHOP_OFF;
	Tx_AD7730(3, gauge);

	//check if settings were stored - read filter reg:
	spi_tx_buffer[0] = CR_SINGLE_READ | CR_FILTER_REGISTER; //see if can just pass pointer to this as arg in function below
	Tx_AD7730(1, gauge);
	Rx_AD7730(3, gauge);

	//-------------- DAC Config -----------------------
	spi_tx_buffer[0] = CR_SINGLE_WRITE | CR_DAC_REGISTER;
	Tx_AD7730(1, gauge);

	spi_tx_buffer[0] = DACR_OFFSET_SIGN_POSITIVE | DACR_OFFSET_NONE; //set DAC to zero offset
	Tx_AD7730(1, gauge);


	//-------------- Internal Full Scale Calibration -----------------------
	spi_tx_buffer[0] = CR_SINGLE_WRITE | CR_MODE_REGISTER;
	Tx_AD7730(1, gauge); //will this just discard data shifted into RX data register?

	spi_tx_buffer[0] = MR1_MODE_INTERNAL_FULL_CALIBRATION | CURRENT_MODE_1_SETTINGS_CAL; //***** TODO: CHECK IF CORRECT VOLTAGE RANGE FOR BIPOLAR 0-10mV?
	spi_tx_buffer[1] = CURRENT_MODE_0_SETTINGS_CAL;
	Tx_AD7730(2, gauge);

	while(HAL_GPIO_ReadPin(gauge.RDY_GPIO_Port, gauge.RDY_Pin) != GPIO_PIN_RESET); //wait for ready pin to go low after calibration

	//-------------- Internal Zero Calibration  -----------------------
	spi_tx_buffer[0] = CR_SINGLE_WRITE | CR_MODE_REGISTER;
	Tx_AD7730(1, gauge); //will this just discard data shifted into RX data register?

	spi_tx_buffer[0] = MR1_MODE_INTERNAL_ZERO_CALIBRATION | CURRENT_MODE_1_SETTINGS_CAL;
	spi_tx_buffer[1] = CURRENT_MODE_0_SETTINGS_CAL;
	Tx_AD7730(2, gauge);

	while(HAL_GPIO_ReadPin(gauge.RDY_GPIO_Port, gauge.RDY_Pin) != GPIO_PIN_RESET); //wait for ready pin to go low after calibration

	//-------------- Store offset obtained from internal calibration -----------------------

	spi_tx_buffer[0] = CR_SINGLE_READ | CR_OFFSET_REGISTER; //see if can just pass pointer to this as arg in function below
	Tx_AD7730(1, gauge); //will this just discard data shifted into RX data register?
	Rx_AD7730(3, gauge);
	gauge.offset_buffer[0] = spi_rx_buffer[0];
	gauge.offset_buffer[1] = spi_rx_buffer[1];
	gauge.offset_buffer[2] = spi_rx_buffer[2]; //***consider using memcpy to do this when cleaning code

	//-------------- Manual Gain setting  -----------------------

	spi_tx_buffer[0] = CR_SINGLE_WRITE | CR_GAIN_REGISTER;
	Tx_AD7730(1, gauge);

	spi_tx_buffer[0] = PGA_GAIN; //set fixed gain for repeatability across experiments
	spi_tx_buffer[1] = 0;
	spi_tx_buffer[2] = 0;
	Tx_AD7730(3, gauge);


	if(SYSTEM_CALIBRATION){
		//-------------- System Full Scale Calibration (apply max load)  -----------------------
		spi_tx_buffer[0] = CR_SINGLE_WRITE | CR_MODE_REGISTER;
		Tx_AD7730(1, gauge); //will this just discard data shifted into RX data register?

		spi_tx_buffer[0] = MR1_MODE_SYSTEM_FULL_CALIBRATION | CURRENT_MODE_1_SETTINGS_CAL;
		spi_tx_buffer[1] = CURRENT_MODE_0_SETTINGS_CAL;
		Tx_AD7730(2, gauge);

		while(HAL_GPIO_ReadPin(gauge.RDY_GPIO_Port, gauge.RDY_Pin) != GPIO_PIN_RESET); //wait for ready pin to go low after calibration

		//-------------- System Zero Scale Calibration (apply zero load)  -----------------------
		spi_tx_buffer[0] = CR_SINGLE_WRITE | CR_MODE_REGISTER;
		Tx_AD7730(1, gauge); //will this just discard data shifted into RX data register?

		spi_tx_buffer[0] = MR1_MODE_SYSTEM_ZERO_CALIBRATION | CURRENT_MODE_1_SETTINGS_CAL;
		spi_tx_buffer[1] = CURRENT_MODE_0_SETTINGS_CAL;
		Tx_AD7730(2, gauge);

		while(HAL_GPIO_ReadPin(gauge.RDY_GPIO_Port, gauge.RDY_Pin) != GPIO_PIN_RESET); //wait for ready pin to go low after calibration
	}

	//-------------- Filter Config for Read Operations (calibration complete) -----------------------
	spi_tx_buffer[0] = CR_SINGLE_WRITE|CR_FILTER_REGISTER; //see if can just pass pointer to this as arg in function below
	Tx_AD7730(1, gauge); //will this just discard data shifted into RX data register?

	//************** TESTING: CHANGING THESE SETTINGS

	spi_tx_buffer[0] = 0x13; //first bits of SF for SF word as 311 (1kHz)
	//spi_tx_buffer[0] = 0x26; //first bits of SF for SF word as 623 (500Hz)
	//spi_tx_buffer[0] = 0x3A; //first bits of SF for SF word as 938 (333Hz)

	//spi_tx_buffer[1] = FR1_SKIP_OFF|FR1_FAST_OFF;
	spi_tx_buffer[1] = 0x70; //bottom bits for 1kHz (SF word 311) with FastMode OFF
	//spi_tx_buffer[1] = 0xF0; //bottom bits for 500Hz (SF word 623) with FastMode OFF
	//spi_tx_buffer[1] = 0xA0; //bottom bits for 333Hz (SF word 938) with FastMode OFF

	spi_tx_buffer[2] = FR0_CHOP_OFF; //******** CHANGE TO OFF IN NORMAL MODE
	Tx_AD7730(3, gauge); //will this just discard data shifted into RX data register?

	//check if settings were stored - read filter reg:
	spi_tx_buffer[0] = CR_SINGLE_READ | CR_FILTER_REGISTER; //see if can just pass pointer to this as arg in function below
	Tx_AD7730(1, gauge); //will this just discard data shifted into RX data register?
	Rx_AD7730(3, gauge);

	HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, 1);

}

void Set_Filter_Reg(AD7730 gauge){
	//TODO: this function is implemented in above - refactor and remove redundancy
	//-------------- Filter Config for Read Operations (calibration complete) -----------------------
		spi_tx_buffer[0] = CR_SINGLE_WRITE|CR_FILTER_REGISTER; //see if can just pass pointer to this as arg in function below
		Tx_AD7730(1, gauge); //will this just discard data shifted into RX data register?

		//************** TESTING: CHANGING THESE SETTINGS

		//spi_tx_buffer[0] = FR2_SINC_AVERAGING_512;
		//spi_tx_buffer[0] = 0x13; //first bits of SF for SF word as 311 (1kHz)
		//spi_tx_buffer[0] = 0x26; //first bits of SF for SF word as 623 (500Hz)
		spi_tx_buffer[0] = 0x3A; //first bits of SF for SF word as 938 (333Hz)

		//spi_tx_buffer[1] = FR1_SKIP_OFF|FR1_FAST_OFF;
		//spi_tx_buffer[1] = 0x70; //bottom bits for 1kHz (SF word 311) with FastMode OFF
		//spi_tx_buffer[1] = 0xF0; //bottom bits for 500Hz (SF word 623) with FastMode OFF
		spi_tx_buffer[1] = 0xA0; //bottom bits for 333Hz (SF word 938) with FastMode OFF

		spi_tx_buffer[2] = FR0_CHOP_OFF; //******** CHANGE TO OFF IN NORMAL MODE
		Tx_AD7730(3, gauge); //will this just discard data shifted into RX data register?
}

/*
 * puts gain from spi_rx_buffer in first byte and offset in second byte of supplied rx_buffer
 */
uint8_t Check_Calibration_Params(AD7730 gauge){
	 //check gain settings:
	  spi_tx_buffer[0] = CR_SINGLE_READ | CR_GAIN_REGISTER; //see if can just pass pointer to this as arg in function below
	  Tx_AD7730(1, gauge); //will this just discard data shifted into RX data register?
	  Rx_AD7730(3, gauge);
	  gauge.data_buffer[0] = spi_rx_buffer[0];

	  spi_tx_buffer[0] = CR_SINGLE_READ | CR_OFFSET_REGISTER; //see if can just pass pointer to this as arg in function below
	  Tx_AD7730(1, gauge); //will this just discard data shifted into RX data register?
	  Rx_AD7730(3, gauge);
	  gauge.data_buffer[1] = spi_rx_buffer[0];

	  if(gauge.data_buffer[0] != PGA_GAIN || gauge.data_buffer[1] != gauge.offset_buffer[0]){
	 		  __asm("BKPT"); //gain not equal to value set earlier?
	 		  return 0;
	  }
	  return 1;
}

/*
 * Sets gain and offsets according to gauge's offset buffer (gain fixed to defined PGA gain at this point)
 */
uint8_t AD7730_Set_Cal_Params(AD7730 gauge){


	spi_tx_buffer[0] = CR_SINGLE_WRITE | CR_OFFSET_REGISTER;
	Tx_AD7730(1, gauge); //will this just discard data shifted into RX data register?

	spi_tx_buffer[0] = gauge.offset_buffer[0]; //this is the only one we care about (channel 1)
	spi_tx_buffer[1] = gauge.offset_buffer[1];
	spi_tx_buffer[2] = gauge.offset_buffer[2];
	Tx_AD7730(3, gauge);

	spi_tx_buffer[0] = CR_SINGLE_WRITE | CR_GAIN_REGISTER;
	Tx_AD7730(1, gauge); //will this just discard data shifted into RX data register?

	spi_tx_buffer[0] = PGA_GAIN;
	spi_tx_buffer[1] = 0;
	spi_tx_buffer[2] = 0;
	Tx_AD7730(3, gauge);


	if(Check_Calibration_Params(gauge)){  //or just return Check_Calibration_Params();
		return 1;
	}
	return 0;
}

/*
 * expects reading and prev readings as percents. Checks for step increase in values synonymous with gain/offset change (error)
 */
uint8_t Verify_Reading(float reading, float prev_reading){
	if(abs(reading - prev_reading) > 80.0){
		return 0; //false for error
	}
	return 1;
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

void AD7730_Read(AD7730 gauge,  uint8_t * rx_buffer){

	// need to put AD7730 into single read mode before every read (returns to idle after every conversion)
	spi_tx_buffer[0] = CR_SINGLE_WRITE | CR_MODE_REGISTER; //write to comms register and set next write to mode reg
	Tx_AD7730(1, gauge); //0x02

	spi_tx_buffer[0] = MR1_MODE_SINGLE | CURRENT_MODE_1_SETTINGS_READ; //write to mode reg startin cont readings for 0-10mV range TODO: check this for bipolar
	spi_tx_buffer[1] = CURRENT_MODE_0_SETTINGS_READ;
	Tx_AD7730(2, gauge);

	spi_tx_buffer[0] = CR_SINGLE_READ | CR_DATA_REGISTER;
	Tx_AD7730(1, gauge);

	spi_tx_buffer[0] = READ_ONLY; //transfer 0xFF (set MSB) to prevent writing to CR accidentally during reading
	spi_tx_buffer[1] = READ_ONLY;
	spi_tx_buffer[2] = READ_ONLY;

	HAL_GPIO_WritePin(gauge.SS_GPIO_Port, gauge.SS_Pin, GPIO_PIN_RESET); //slave select low to begin
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)&spi_tx_buffer, rx_buffer, 3, 1000); //assume this will receive the bytes too?
	HAL_GPIO_WritePin(gauge.SS_GPIO_Port, gauge.SS_Pin, GPIO_PIN_SET); //slave select low to begin
}

/*
 * Config AD7730 for cont read mode. Note: should be prepared to take readings at specified rate straight after invocation of this fn
 */
void AD7730_Start_Cont_Read(AD7730 gauge){

	spi_tx_buffer[0] = 0x02;//CR_SINGLE_WRITE | CR_MODE_REGISTER; //write to comms register and set next write to mode reg
	Tx_AD7730(1, gauge); //0x02

	spi_tx_buffer[0] = 0x21;//MR1_MODE_CONTINUOUS | CURRENT_MODE_1_SETTINGS_READ; //write to mode reg startin cont readings for 0-10mV range TODO: check this for bipolar
	spi_tx_buffer[1] = 0x80; //CURRENT_MODE_0_SETTINGS_READ;
	Tx_AD7730(2, gauge); //0x2180

	spi_tx_buffer[0] = 0x21; //CR_CONTINUOUS_READ_START | CR_DATA_REGISTER;
	Tx_AD7730(1, gauge); //0x21

	while(HAL_GPIO_ReadPin(gauge.RDY_GPIO_Port, gauge.RDY_Pin) != GPIO_PIN_RESET); //wait for ready pin to go low when cont readings start

	CONT_READ_STARTED = SET; //TODO: will this work for all gauges? first gauge can set it but subsequent ones don't need to..
}

void AD7730_Read_Cont(AD7730 gauge, uint8_t * rx_buffer){
		spi_tx_buffer[0] = 0x0;
		spi_tx_buffer[1] = 0x0;
		spi_tx_buffer[2] = 0x0; //need to transfer all 0 bytes to keep DIN of AD7730 low prevent device reset during cont read

		while(HAL_GPIO_ReadPin(gauge.RDY_GPIO_Port, gauge.RDY_Pin) != GPIO_PIN_RESET); //wait for ready pin to go low

		HAL_GPIO_WritePin(gauge.SS_GPIO_Port, gauge.SS_Pin, GPIO_PIN_RESET); //slave select low to begin
		if(HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)&spi_tx_buffer, rx_buffer, 3, 1000) != HAL_OK){
			__asm("BKPT");
		}
		HAL_GPIO_WritePin(gauge.SS_GPIO_Port, gauge.SS_Pin, GPIO_PIN_SET); //slave select low to begin
}

/*
 * Stop cont read. Returns to idle state
 */
void AD7730_Stop_Cont_Read(AD7730 gauge){
	spi_tx_buffer[0] = CR_SINGLE_WRITE | CR_CONTINUOUS_READ_STOP; //0x30
	Tx_AD7730(1, gauge);
	CONT_READ_STARTED = RESET;
}

/*
 * Expects 3 data bytes in rx_buffer. See AD7730 manual - in bipolar mode
 * full scale negative: 0000...000
 * zero differential: 10000...000
 * full scale positive: 1111...111 (so can consider as unsigned 32 bit int)
 */
int32_t AD7730_Process_Reading_Num(AD7730 gauge){
	uint32_t filler =0;
	uint32_t result = filler | ((gauge.data_buffer[0] << 16) | (gauge.data_buffer[1] << 8) | gauge.data_buffer[2]);
	result = result - 0x800000; //so that 0 will no longer be 100..00 but actually 0
	if(DIFFERENTIAL_POLARITY_SWITCH){
		result *= -1;
	}
	return result;
}

/*
 * Expects data in spi_rx_buffer. See AD7730 manual - in bipolar mode
 * full scale negative: 0000...000
 * zero differential: 10000...000
 * full scale positive: 1111...111 (so can consider as unsigned 32 bit int)
 */
float AD7730_Process_Reading_Percent(AD7730 gauge){
	int32_t reading = AD7730_Process_Reading_Num(gauge);
	float x = (float)reading;
	return (x/(float)0x7FFFFF) * 100; //divide by max full scale voltage
}

void AD7730_Reset(void){
	HAL_GPIO_WritePin(AD7730_RESET_PORT, AD7730_RESET_PIN, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(AD7730_RESET_PORT, AD7730_RESET_PIN, GPIO_PIN_SET);
}

//Haven't tested this
void AD7730_Sync(void){
	HAL_GPIO_WritePin(AD7730_SYNC_PORT, AD7730_SYNC_PIN, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(AD7730_SYNC_PORT, AD7730_SYNC_PIN, GPIO_PIN_SET);
}

/*
 * Reset a chip through software. Needs DIN line high for at least 32 clock cycles (min 4 bytes)
 */
void AD7730_soft_reset(AD7730 gauge){
	spi_tx_buffer[0] = 0xFF;
	spi_tx_buffer[1] = 0xFF;
	spi_tx_buffer[2] = 0xFF;

	for(char i = 0; i < 2; i++){  //Transmit 9 bytes of 0xFF to reset
		Tx_AD7730(3, gauge);
	}

}

void delayUS(uint32_t us) {
	volatile uint32_t counter = 21*us; //tested on scope (102 vs 100 us)
	while(counter--);
}
