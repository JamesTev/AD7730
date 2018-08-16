#include "stm32f4xx_hal.h"

#define DMA_RX_BUFFER_SIZE          32
uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];

#define UART_BUFFER_SIZE            256
uint8_t UART_Buffer[UART_BUFFER_SIZE];

uint32_t test_cnt;

void USART_IrqHandler (UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma);
void DMA_IrqHandler (DMA_HandleTypeDef *hdma);
void Config_Idle_IRQ(void);


