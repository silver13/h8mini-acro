
#include "gd32f1x0.h"
#include <stdio.h>
#include "drv_serial.h"
#include "config.h"

#define SERIAL_BUFFER_SIZE 64

#ifdef SERIAL 

uint8_t buffer[SERIAL_BUFFER_SIZE];
char buffer_start = 0;
char buffer_end = 1;


int fputc(int ch, FILE *f)
{	
	buffer[buffer_end] =(char) ch;
	buffer_end++;
	buffer_end=buffer_end%(SERIAL_BUFFER_SIZE);
	NVIC_EnableIRQ( USART2_IRQn);
  return ch;
}



void USART2_IRQHandler(void)
{
	if ( buffer_end != buffer_start  ) 
	{
		USART_DataSend( USART2 , buffer[buffer_start] );
		buffer_start++;
		buffer_start=buffer_start%(SERIAL_BUFFER_SIZE);
	}
	else
	{
		 NVIC_DisableIRQ( USART2_IRQn);
	}
}

void serial_init(void)
{

		
    //RCC_AHBPeriphClock_Enable( RCC_AHBPERIPH_GPIOA , ENABLE );   

    RCC_APB1PeriphClock_Enable( RCC_APB1PERIPH_USART2 , ENABLE );
	
		GPIO_InitPara GPIO_InitStructure;

		GPIO_PinAFConfig( GPIOA , GPIO_PINSOURCE2, GPIO_AF_1 );    
		GPIO_PinAFConfig( GPIOA , GPIO_PINSOURCE3, GPIO_AF_1 ); 

		GPIO_InitStructure.GPIO_Pin     = GPIO_PIN_2 | GPIO_PIN_3;
		GPIO_InitStructure.GPIO_Mode    = GPIO_MODE_AF;
		GPIO_InitStructure.GPIO_Speed   = GPIO_SPEED_50MHZ;
		GPIO_InitStructure.GPIO_OType   = GPIO_OTYPE_PP;
		GPIO_InitStructure.GPIO_PuPd    = GPIO_PUPD_NOPULL;
		GPIO_Init( GPIOA , &GPIO_InitStructure); 



		USART_InitPara USART_InitStructure;
		
		USART_InitStructure.USART_BRR           = 57600;
		USART_InitStructure.USART_WL            = USART_WL_8B;
		USART_InitStructure.USART_STBits        = USART_STBITS_1;
		USART_InitStructure.USART_Parity        = USART_PARITY_RESET;
		USART_InitStructure.USART_HardwareFlowControl = USART_HARDWAREFLOWCONTROL_NONE;
		USART_InitStructure.USART_RxorTx        = USART_RXORTX_TX;
		USART_Init(USART2, &USART_InitStructure);

    USART_Enable(USART2, ENABLE);
		
	  USART_INT_Set(USART2, USART_INT_TC , ENABLE); // USART_INT_TBE
	


}
#endif

