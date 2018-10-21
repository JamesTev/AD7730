/*
 * SerialTerminal.c
 *
 *  Created on: Jul 4, 2012
 *  Author: James Gowans
 *  Modified by Callen Fisher and James Teversham 2018
 */
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

  //put end char
  pkt_to_tx.data[packet_data_pointer++] = 0x7E;

  //set pkt length
  pkt_to_tx.bytes_to_tx = packet_data_pointer;

  return pkt_to_tx;
}

/* Function to reverse 32bit array from start to end
 * Note: should write to the array passed in (doesn't return new one)
 */
void reverseArray(uint32_t * arr, int len)
{
    int temp;
    int start = 0;
    int end = len-1;
    while (start < end)
    {
        temp = arr[start];
        arr[start] = arr[end];
        arr[end] = temp;
        start++;
        end--;
    }
}
