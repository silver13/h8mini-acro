//
#include "gd32f1x0.h"
#include "drv_time.h"

void failloop( int val);

unsigned long lastticks;
unsigned long globalticks;
volatile unsigned long systickcount = 0;

// systick interrupt routine
void SysTick_Handler(void)
{
    systickcount++;
}


void time_init()
{
	// 72000000 / (8 (internal div) * 9 ) 
  if (SysTick_Config(SystemCoreClock / (9) )) //1sec interrupts
    {// not able to set divider
			  failloop(8);
        while (1);
    }
    NVIC_SetPriority(SysTick_IRQn, 0x00);	
}

// called at least once per second or time will overflow
unsigned long time_update(void)
{
unsigned long maxticks = SysTick->LOAD;	
unsigned long ticks = SysTick->VAL;	
unsigned long elapsedticks;	

	if (ticks < lastticks) elapsedticks = lastticks - ticks;	
	else
	{// overflow ( underflow really)
	elapsedticks = lastticks + ( maxticks - ticks);			
	}
	
lastticks = ticks;
globalticks = globalticks+ elapsedticks/8;
return globalticks;	
}


// return time in uS from start ( micros())
unsigned long gettime()
{
unsigned long time = time_update();
//time = time/1;
return time;		
}


void delay(uint32_t data)
{
    while(data--);
}


