/*
 * opto_force.c
 *
 *  Created on: 02 Aug 2018
 *      Author: jamesteversham
 */
#include "opto_force.h"
#include "main.h"
#include "stm32f4xx_hal.h"

uint8_t OF_config[9] = {170, 0, 50, 3, sample_1kHz, filter_500Hz, zeroing, 1, 224};
uint8_t OF_Config_Complete = 0;
//uint8_t OF_config[9] = {170, 0, 50, 3, sample_10Hz, no_filter, zeroing, 1, 224};

UART_HandleTypeDef huart3; //used for sampling OF sensor
UART_HandleTypeDef huart2;
uint8_t ack_buffer[7];

void Config_OF(void){
	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, 1);

	uint8_t tx_config = HAL_UART_Transmit(&huart3, (uint8_t *)&OF_config, OF_config_buffer_size, 5000);
	uint8_t rx_ack = HAL_UART_Receive_DMA(&huart3, ((uint8_t *)&ack_buffer), 7); //explicit casting to avoid compiler warnings

	if((tx_config | rx_ack) != HAL_OK){
		_Error_Handler(__FILE__, __LINE__);
	}
}

/**
  * @brief Validates header bytes, error bytes and checksum of received OF packet. Accepts pointer to rx buffer
  * @retval 1 (valid) or 0 (invalid/error)
  */
uint8_t Validate_OF_Data_Packet(uint8_t * buffer, uint8_t packet_type){
	uint32_t header = (buffer[0]<<24) | (buffer[1]<<16) | (buffer[2]<<8) | buffer[3];
	uint32_t expected_header = 0xAA07080A; //default to data packet header and packet size
	uint8_t packet_size = 16;
	uint8_t status_byte = buffer[6]|buffer[7];
	if(packet_type == OF_CONFIG_PACKET){
		expected_header = 0xAA005001; //verify this is the header corresponding to 170 0 80 1
		packet_size = 7;
		status_byte = buffer[4];
	}
	if(header!=expected_header){ //every packet has header bytes 170, 7, 8, 10 for data and 170, 0, 80, 1 for config
		return OF_HEADER_ERROR;
	}

	//check two status bytes. Should both be 0 if no errors in packet
	if(status_byte !=0){
		return OF_STATUS_ERROR;
	}
	uint16_t checksum = 0;
	for(uint8_t i = 0; i<packet_size-2; i++){
		checksum += buffer[i];
	}
	//check sum
	if(checksum!=((buffer[packet_size-2]<<8) | buffer[packet_size-1])){
		return OF_CHECKSUM_ERROR;
	}
	if(packet_type == OF_CONFIG_PACKET){
		OF_Config_Complete = 1;
	}
	return OF_PACKET_VALID;
}


