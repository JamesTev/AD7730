/*
 * Manual circular DMA implementation. Needed greater control than that offered by HAL implementation.
 * Uses UART idle line interrupt. Note the manual call to DMA RX Complete callback
 */
#include "dma_circular.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;


extern uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];
extern uint8_t UART_Buffer[UART_BUFFER_SIZE];

size_t Write;
size_t len, tocopy;
uint8_t* ptr;

uint8_t trigger_cnt = 0;
uint32_t test_cnt = 0;

void Config_Idle_IRQ(void){
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);   // enable idle line interrupt
	__HAL_DMA_ENABLE_IT(&hdma_usart3_rx, DMA_IT_TC);  // enable DMA Tx cplt interrupt
	hdma_usart3_rx.Instance->CR &= ~DMA_SxCR_HTIE;  // disable uart half tx interrupt
}

void USART_IrqHandler (UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma)
{
	if (huart->Instance->SR & UART_FLAG_IDLE)           /* if Idle flag is set */
	{
		volatile uint32_t tmp;                  /* Must be volatile to prevent optimizations */
        tmp = huart->Instance->SR;                       /* Read status register */
        tmp = huart->Instance->DR;                       /* Read data register */
		hdma->Instance->CR &= ~DMA_SxCR_EN;       /* Disabling DMA will force transfer complete interrupt if enabled */       
	}
}

void DMA_IrqHandler (DMA_HandleTypeDef *hdma)
{
	typedef struct
	{
		__IO uint32_t ISR;   /*!< DMA interrupt status register */
		__IO uint32_t Reserved0;
		__IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
	} DMA_Base_Registers;

	DMA_Base_Registers *regs = (DMA_Base_Registers *)hdma->StreamBaseAddress;
	
	if(__HAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_TC) != RESET)   // if the source is TC
	{
		trigger_cnt++;
		/* Clear the transfer complete flag */
      regs->IFCR = DMA_FLAG_TCIF0_4 << hdma->StreamIndex;
	  
	     /* Get the length of the data */
	  len = DMA_RX_BUFFER_SIZE - hdma->Instance->NDTR;  
	  
	  /* Get number of bytes we can copy to the end of buffer */
	  tocopy = UART_BUFFER_SIZE - Write;
	  
	  /* Check how many bytes to copy */
       if (tocopy > len) 
       {
            tocopy = len;
        }
		
		 /* Write received data for UART main buffer for manipulation later */
        ptr = DMA_RX_Buffer;
        memcpy(&UART_Buffer[Write], ptr, tocopy);   /* Copy first part */
				
		/* Correct values for remaining data */
        Write += tocopy;
        len -= tocopy;
        ptr += tocopy;

		/* If still data to write for beginning of buffer */
       if (len) 
		{
            memcpy(&UART_Buffer[0], ptr, len);      /* Don't care if we override Read pointer now */
            Write = len;

         }
		
		/* Prepare DMA for next transfer */
        /* Important! DMA stream won't start if all flags are not cleared first */

       if(hdma->Instance->NDTR == 0){
    	   HAL_UART_RxCpltCallback(&huart3);
        }
 
        regs->IFCR = 0x3FU << hdma->StreamIndex; // clear all interrupts
		hdma->Instance->M0AR = (uint32_t)DMA_RX_Buffer;   /* Set memory address for DMA again */
        hdma->Instance->NDTR = 16;//DMA_RX_BUFFER_SIZE;    /* Set number of bytes to receive */
        if(trigger_cnt == 2){ //this interrupt gets triggered on every idle line - only full if 16 bytes of data received.
        	trigger_cnt = 0;
        }

        hdma->Instance->CR |= DMA_SxCR_EN;            /* Start DMA transfer */
	}
}	
