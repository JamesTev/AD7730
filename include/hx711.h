#ifndef HX711_H_
#define HX711_H_
// modified from https://github.com/freakone/HX711
#include "stm32f4xx.h"

typedef struct _hx711
{
	GPIO_TypeDef* gpioSck; //port of SCK
	GPIO_TypeDef* gpioData; //port of Data
	uint16_t pinSck;
	uint16_t pinData;
	int offset;
	int gain;
} HX711;

enum gain{
	GAIN_128 = 1,
	GAIN_32 = 2,
	GAIN_64 = 3
};

void HX711_Init(HX711 data);
int32_t HX711_Value(HX711 data);



#endif /* HX711_H_ */
