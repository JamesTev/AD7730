/*
 * AD7730.h
 *
 *  Created on: 02 Aug 2018
 *      Author: jamesteversham
 */
#include "stm32f4xx.h"

#ifndef AD7730_H_
#define AD7730_H_

#define SYSTEM_CALIBRATION 0
#define DIFFERENTIAL_POLARITY_SWITCH 1
#define PGA_GAIN 130

#define GAUGE1_RDY_PORT GPIOE
#define GAUGE1_RDY_PIN	GPIO_PIN_7
#define GAUGE1_SS_PORT GPIOE
#define GAUGE1_SS_PIN GPIO_PIN_9

#define GAUGE2_RDY_PORT GPIOE
#define GAUGE2_RDY_PIN	GPIO_PIN_11
#define GAUGE2_SS_PORT GPIOE
#define GAUGE2_SS_PIN GPIO_PIN_13

#define GAUGE3_RDY_PORT GPIOE
#define GAUGE3_RDY_PIN	GPIO_PIN_8
#define GAUGE3_SS_PORT GPIOE
#define GAUGE3_SS_PIN GPIO_PIN_10

#define GAUGE4_RDY_PORT GPIOE
#define GAUGE4_RDY_PIN	GPIO_PIN_12
#define GAUGE4_SS_PORT GPIOE
#define GAUGE4_SS_PIN GPIO_PIN_14

#define AD7730_RESET_PORT GPIOE
#define AD7730_RESET_PIN GPIO_PIN_15

#define AD7730_SYNC_PORT GPIOB
#define AD7730_SYNC_PIN GPIO_PIN_12

#define RESET 8
#define RDY 9
#define DEBUG_AD7730 0
#define SPI_DELAY 10

#define READ_ONLY 0xFF
//Communication Register Values
#define CR_SINGLE_WRITE 0x00
#define CR_SINGLE_READ 0x10
#define CR_CONTINUOUS_READ_START 0x20
#define CR_CONTINUOUS_READ_STOP 0x30

#define CR_COMMUNICATION_REGISTER 0x00 //Write only
#define CR_STATUS_REGISTER 0x00 //Read only
#define CR_DATA_REGISTER 0x01
#define CR_MODE_REGISTER 0x02
#define CR_FILTER_REGISTER 0x03
#define CR_DAC_REGISTER 0x04
#define CR_OFFSET_REGISTER 0x05
#define CR_GAIN_REGISTER 0x06
#define CR_TEST_REGISTER 0x07 //these registers have all been verified with data sheet

//Mode Register Values
#define MR1_MODE_IDLE 0x00
#define MR1_MODE_CONTINUOUS 0x20 //Standard Operation
#define MR1_MODE_SINGLE 0x40
#define MR1_MODE_STANDBY 0x60
#define MR1_MODE_INTERNAL_ZERO_CALIBRATION 0x80
#define MR1_MODE_INTERNAL_FULL_CALIBRATION 0xA0
#define MR1_MODE_SYSTEM_ZERO_CALIBRATION 0xC0
#define MR1_MODE_SYSTEM_FULL_CALIBRATION 0xE0
#define MR1_BU_BIPOLAR 0x00 //+- voltage defined by MR0_RANGE
#define MR1_BU_UNIPOLAR 0x10 //0 to voltage deifined by MRO_RANGE
#define MR1_WL_24_BIT 0x01
#define MR1_WL_16_BIT 0x00

#define MR0_HIREF_5V 0x80
#define MR0_HIREF_2P5V 0x00
#define MR0_RANGE_10MV 0x00
#define MR0_RANGE_20MV 0x01
#define MR0_RANGE_40MV 0x02
#define MR0_RANGE_80MV 0x03
#define MR0_CHANNEL_1 0x00
#define MR0_CHANNEL_2 0x01
#define MR0_CHANNEL_SHORT_1 0x02 //Used for internal noise check
#define MR0_CHANNEL_NEGATIVE_1_2 0x03 //Unknown use
#define MRO_BURNOUT_ON 0x04 //Advanced, to check if loadcell is burnt out

//Filter Register Values
#define FR2_SINC_AVERAGING_2048 0x80  //Base sample rate of 50 Hz
#define FR2_SINC_AVERAGING_1024 0x40  //Base sample rate of 100 Hz
#define FR2_SINC_AVERAGING_512 0x20   //Base sample rate of 200 Hz
#define FR2_SINC_AVERAGING_256 0x10   //Base sample rate of 400 Hz

#define FR1_SKIP_ON 0x02 //the FIR filter on the part is bypassed
#define FR1_SKIP_OFF 0x00
#define FR1_FAST_ON 0x01 //FIR is replaced with moving average on large step, sinc filter averages are used to compensate
#define FR1_FAST_OFF 0x00

#define FR0_CHOP_ON 0x10 //When the chop mode is enabled, the part is effectively chopped at its input and output to remove all offset and offset drift errors on the part.
#define FR0_CHOP_OFF 0x00 //Increases sample rate by x3

//DAC Register Values
#define DACR_OFFSET_SIGN_POSITIVE 0x00
#define DACR_OFFSET_SIGN_NEGATIVE 0x20
#define DACR_OFFSET_40MV 0x10
#define DACR_OFFSET_20MV 0x08
#define DACR_OFFSET_10MV 0x04
#define DACR_OFFSET_5MV 0x02
#define DACR_OFFSET_2P5MV 0x01
#define DACR_OFFSET_NONE 0x00

//current settings
#define CURRENT_MODE_1_SETTINGS_CAL (MR1_BU_BIPOLAR | MR1_WL_24_BIT)
#define CURRENT_MODE_0_SETTINGS_CAL (MR0_HIREF_5V | MR0_RANGE_10MV | MR0_CHANNEL_1) //datasheet recommends calibrating with 80mV range regardless

#define CURRENT_MODE_1_SETTINGS_READ (MR1_BU_BIPOLAR | MR1_WL_24_BIT)
#define CURRENT_MODE_0_SETTINGS_READ (MR0_HIREF_5V | MR0_RANGE_10MV | MR0_CHANNEL_1)

volatile uint8_t spi_tx_buffer[3]; //only need to transfer at most 3 bytes during config
volatile uint8_t spi_rx_buffer[3];

volatile uint8_t gauge1_data_buffer[3];
volatile uint8_t gauge2_data_buffer[3];
volatile uint8_t gauge3_data_buffer[3];
volatile uint8_t gauge4_data_buffer[3]; //do we need to define these in a source file?

typedef struct _ad7730
{
	GPIO_TypeDef* SS_GPIO_Port; //port of SCK
	GPIO_TypeDef* RDY_GPIO_Port; //port of SCK
	uint16_t SS_Pin;
	uint16_t RDY_Pin;

} AD7730;


void AD7730_Init(AD7730 gauge);
void AD7730_Config(AD7730 gauge);
void Tx_AD7730(uint16_t length, AD7730 gauge);
void Rx_AD7730(uint16_t length, AD7730 gauge);
void AD7730_start_cont_read(void);
void AD7730_Read(AD7730 gauge, uint8_t * rx_buffer);
void AD7730_Start_Cont_Read(AD7730 gauge);
void AD7730_Read_Cont(AD7730 gauge, uint8_t * rx_buffer);
void AD7730_Stop_Cont_Read(AD7730 gauge);
void AD7730_Reset(void);
void delayUS(uint32_t us);
int32_t AD7730_Process_Reading_Num(uint8_t * rx_buffer);
float AD7730_Process_Reading_Percent(uint8_t * rx_buffer);

#endif /* AD7730_H_ */
