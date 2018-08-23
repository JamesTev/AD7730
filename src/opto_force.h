/*
 * opto_force.h
 *
 *  Created on: 02 Aug 2018
 *      Author: jamesteversham
 */

#ifndef OPTO_FORCE_H_
#define OPTO_FORCE_H_

#include "stm32f4xx_hal.h"

#define OF_config_buffer_size 9
#define OF_buffer_size 192 //opto force receive buffer size


#define sample_1kHz 1
#define sample_333Hz 3
#define sample_10Hz 100

#define filter_500Hz 1
#define filter_150Hz 2

#define no_filter 0
#define zeroing 255

#define OF_HEADER_ERROR 1
#define OF_STATUS_ERROR 2
#define OF_CHECKSUM_ERROR 3
#define OF_PACKET_VALID 0

#define OF_DATA_PACKET 0
#define OF_CONFIG_PACKET 1

uint8_t ack_buffer[7];
uint8_t OF_Config_Complete;

extern uint8_t OF_config[OF_config_buffer_size];
void Config_OF(void);
uint8_t Validate_OF_Data_Packet(uint8_t * buffer, uint8_t packet_type);

#endif /* OPTO_FORCE_H_ */
