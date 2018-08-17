/*
 * SerialTerminal.c
 *
 *  Created on: Jul 4, 2012
 *      Author: James Gowans
 */
//modified by callen fisher and James Teversham 2018
#include "serial_terminal.h"

CRC_HandleTypeDef hcrc;

CommsTask_TransmitPacketStruct serialTerminal_packetize(uint8_t* payload_to_pack, uint16_t length_of_payload)
{
  CommsTask_TransmitPacketStruct pkt_to_tx;
  uint16_t raw_data_pointer = 0;
  uint16_t packet_data_pointer = 0;
  uint32_t crcCalculated;
  uint8_t char_to_pack;

  //put start char
  pkt_to_tx.data[packet_data_pointer++] = 0x7E; //0x7E = '~'

  //put data
  for (raw_data_pointer = 0; raw_data_pointer < length_of_payload; raw_data_pointer++, packet_data_pointer++)
  { //for both bytes of the pkt_code,
    char_to_pack = payload_to_pack[raw_data_pointer];
    if (char_to_pack == 0x7E)
    {
      pkt_to_tx.data[packet_data_pointer++] = 0x7D;
      pkt_to_tx.data[packet_data_pointer] = 0x5E;
    }
    else if (char_to_pack == 0x7D)
    {
      pkt_to_tx.data[packet_data_pointer++] = 0x7D;
      pkt_to_tx.data[packet_data_pointer] = 0x5D;
    }
    else
    {
      pkt_to_tx.data[packet_data_pointer] = char_to_pack;
    }
  }

  //pad with required number of zeros to make packet a multiple of 4 (hardware CRC must operate on sequence of words/32 bits)
  uint8_t filler_bytes = ((packet_data_pointer-1)/4+1)*4 - (packet_data_pointer-1);
  for(uint8_t i=packet_data_pointer; i < packet_data_pointer+filler_bytes; i++){
    pkt_to_tx.data[i] =0;
  }
  packet_data_pointer+= filler_bytes;

  /*
   * Generate 32 bit CRC using hardware CRC generator. Exclude start char (~) from CRC source so start from second element
   */
//  crcCalculated = HAL_CRC_Calculate(&hcrc, (uint8_t *)&pkt_to_tx.data[1], packet_data_pointer - 1); //generator poly 0x4C11DB7
//  //crcCalculated = (uint16_t) crcCalc(payload_to_pack, 0, length_of_payload);
//
//  //put CRC
//  for (raw_data_pointer = 0; raw_data_pointer < 4; raw_data_pointer++, packet_data_pointer++)
//  {
//    char_to_pack = (uint8_t) ((crcCalculated >> 8 * (3 - raw_data_pointer)) & 0x00FF);
//    if (char_to_pack == 0x7E)
//    {
//      pkt_to_tx.data[packet_data_pointer++] = 0x7D;
//      pkt_to_tx.data[packet_data_pointer] = 0x5E;
//    }
//    else if (char_to_pack == 0x7D)
//    {
//      pkt_to_tx.data[packet_data_pointer++] = 0x7D;
//      pkt_to_tx.data[packet_data_pointer] = 0x5D;
//    }
//    else
//    {
//      pkt_to_tx.data[packet_data_pointer] = char_to_pack;
//    }
//  }

  //put end char
  pkt_to_tx.data[packet_data_pointer++] = 0x7E;

  //set pkt length
  pkt_to_tx.bytes_to_tx = packet_data_pointer;

  return pkt_to_tx;
}
