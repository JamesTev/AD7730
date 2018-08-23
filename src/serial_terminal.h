/*
 * SerialTerminal.h
 *
 *  Created on: Jul 4, 2012
 *      Origianl Author: James Gowans
 *      Adapted by James Teversham 2018
 *
 *      Description:  Provides functionality for printf and console via UART
 */

#ifndef SERIALTERMINAL_H_
#define SERIALTERMINAL_H_

#include "stm32f4xx.h"
#include <stdlib.h>
#include "CRC.h"

#define COMMS_TX_BUFFER_SIZE 34

typedef struct {
  uint8_t data[COMMS_TX_BUFFER_SIZE];
  uint16_t bytes_to_tx;
} CommsTask_TransmitPacketStruct;

void reverseArray(uint32_t * arr, int len);

CommsTask_TransmitPacketStruct serialTerminal_packetize(uint8_t* payload_to_pack, uint16_t length_of_payload);
#endif /* SERIALTERMINAL_H_ */
