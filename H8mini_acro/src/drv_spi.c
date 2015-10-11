

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
	
	spi_csoff();
}


#define gpioset( port , pin) port->BOR = (0x0001 << pin)
#define gpioreset( port , pin) port->BCR = (0x0001 << pin)

#define MOSIHIGH gpioset( GPIOB, 3)
#define MOSILOW gpioreset( GPIOB, 3);
#define SCKHIGH gpioset( GPIOB, 4);
#define SCKLOW gpioreset( GPIOB, 4);

#define READMISO ((GPIOA->DIR & GPIO_PIN_15) != (uint32_t)Bit_RESET)


void spi_cson( )
{
	GPIO_WriteBit(GPIOB, GPIO_PIN_5, Bit_RESET);
}

void spi_csoff( )
{
	gpioset( GPIOB, 5);
}


void spi_sendbyte ( int data)
{
for ( int i =7 ; i >=0 ; i--)
	{
		if ( bitRead( data , i)  ) 
		{
			MOSIHIGH;
		}
		else 
		{
			MOSILOW;
		}
	
		SCKHIGH;
		SCKLOW;
	}
}


int spi_sendrecvbyte ( int data)
{ int recv = 0;
	for ( int i =7 ; i >=0 ; i--)
	{
		if ( (data) & (1<<7)  ) 
		{
			MOSIHIGH;
		}
		else 
		{
			MOSILOW;
		}
		SCKHIGH;
		data = data<<1;
		if ( READMISO ) recv= recv|(1<<7);
		recv = recv<<1;
		SCKLOW;
	}	
	  recv = recv>>8;
    return recv;
}

















