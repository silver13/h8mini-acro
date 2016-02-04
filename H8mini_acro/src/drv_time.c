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
//    systickcount++;
}

 // divider by 8 is enabled in this systick config                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
static __INLINE uint32_t SysTick_Config2(uint32_t ticks)
{ 
  if (ticks > SysTick_LOAD_RELOAD_Msk)  return (1);            /* Reload value impossible */
                                                               
  SysTick->LOAD  = (ticks & SysTick_LOAD_RELOAD_Msk) - 1;      /* set reload register */
  NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority for Cortex-M0 System Interrupts */
  SysTick->VAL   = 0;                                          /* Load the SysTick Counter Value */
  SysTick->CTRL  = //SysTick_CTRL_CLKSOURCE_Msk |   // divide by 8
                   SysTick_CTRL_TICKINT_Msk   | 
                   SysTick_CTRL_ENABLE_Msk;                    /* Enable SysTick IRQ and SysTick Timer */
  return (0);                                                  /* Function successful */
}


void time_init()
{

	  if (SysTick_Config2( 48000000/8 ))
    {// not able to set divider
			  failloop(5);
        while (1);
    }
   // NVIC_SetPriority(SysTick_IRQn, 0x00);	
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
globalticks = globalticks+ elapsedticks/6;
return globalticks;	
}


// return time in uS from start ( micros())
unsigned long gettime()
{
unsigned long time = time_update();
return time;		
}


void delay(uint32_t data)
{
	volatile uint32_t count;
	count = data * 16;
	while (count--);
}
