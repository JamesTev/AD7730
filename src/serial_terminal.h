/*
 * SerialTerminal.h
 *
 *  Created on: Jul 4, 2012
 *      Author: James Gowans
 *
 *      Description:  Provides functionality for printf and console via UART
 */

//Modified by James Teversham and callen fisher 2018
#ifndef SERIALTERMINAL_H_
#define SERIALTERMINAL_H_

#include "stm32f4xx.h"
#include <stdlib.h>
#include "CRC.h"

#define COMMS_TX_BUFFER_SIZE 30

typedef struct {
  uint8_t data[COMMS_TX_BUFFER_SIZE - 2];
  uint16_t bytes_to_tx;
} CommsTask_TransmitPacketStruct;

CommsTask_TransmitPacketStruct serialTerminal_packetize(uint8_t* payload_to_pack, uint16_t length_of_payload);
#endif /* SERIALTERMINAL_H_ */
