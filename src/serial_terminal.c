/*
 * SerialTerminal.c
 *
 *  Created on: Jul 4, 2012
 *      Author: James Gowans
 */
//modified by callen fisher 2018
#include "serial_terminal.h"

void serialTerminal_Init(void)
{
  initCRC();
}

CommsTask_TransmitPacketStruct serialTerminal_packetize(uint8_t* payload_to_pack, uint16_t length_of_payload)
{
  CommsTask_TransmitPacketStruct pkt_to_tx;
  uint16_t raw_data_pointer = 0;
  uint16_t packet_data_pointer = 0;
  uint16_t crcCalculated;
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

//  //calculate CRC
//  crcCalculated = (uint16_t) crcCalc(payload_to_pack, 0, length_of_payload);
//
//  //put CRC
//  for (raw_data_pointer = 0; raw_data_pointer < 2; raw_data_pointer++, packet_data_pointer++)
//  {
//    char_to_pack = (uint8_t) ((crcCalculated >> 8 * (1 - raw_data_pointer)) & 0x00FF);
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
