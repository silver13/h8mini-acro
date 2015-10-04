

#include "gd32f1x0.h"
#include "drv_spi.h"
#include "macros.h"

void spi_init(void)
{    
	GPIO_InitPara GPIO_InitStructure;

 // RCC_AHBPeriphClock_Enable( RCC_AHBPERIPH_GPIOA | RCC_AHBPERIPH_GPIOB  , ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_3| GPIO_PIN_4 | GPIO_PIN_5  ;
	GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
	GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OTYPE_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_PULLUP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_MODE_IN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
}


#define gpioset( port , pin) port->BOR = (0x0001 << pin)
#define gpioreset( port , pin) port->BCR = (0x0001 << pin)

void mosihigh( )
{
	//GPIO_WriteBit(GPIOB, GPIO_PIN_3, Bit_SET);
	//GPIOB->BOR = GPIO_PIN_3;
	gpioset( GPIOB, 3);
}

void sckhigh( )
{
	//GPIO_WriteBit(GPIOB, GPIO_PIN_4, Bit_SET);
  //GPIOB->BOR = GPIO_PIN_4;
  gpioset( GPIOB, 4);	
}


void scklow( )
{
//GPIO_WriteBit(GPIOB, GPIO_PIN_4, Bit_RESET);	
	//GPIOB->BCR = GPIO_PIN_4;
	gpioreset( GPIOB, 4);
}

void mosilow( )
{
	//GPIO_WriteBit(GPIOB, GPIO_PIN_3, Bit_RESET);
	//GPIOB->BCR = GPIO_PIN_3;
	gpioreset( GPIOB, 3);
}
void spi_cson( )
{
	GPIO_WriteBit(GPIOB, GPIO_PIN_5, Bit_RESET);
	//gpioset( GPIOB, 5);
}

void spi_csoff( )
{
	GPIO_WriteBit(GPIOB, GPIO_PIN_5, Bit_SET);
	//gpioset( GPIOB, 5);
}


int readmiso()
{
	//return GPIO_ReadInputBit( GPIOA, GPIO_PIN_15)	;
	return ((GPIOA->DIR & GPIO_PIN_15) != (uint32_t)Bit_RESET);
}


void spi_sendbyte ( uint8_t data)
{
for ( int i =7 ; i >=0 ; i--)
	{
		if ( bitRead( data , i)  ) mosihigh( );
		else mosilow( );
		sckhigh();
		scklow();
	}
}

uint8_t spi_sendrecvbyte ( uint8_t data)
{ int recv = 0;
	for ( int i =7 ; i >=0 ; i--)
	{
		if ( bitRead( data , i)  ) mosihigh( );
		else mosilow( );
		sckhigh();
		if ( readmiso()  ) bitSet ( recv , i);	
		scklow();
	}	
    return recv;
}


















