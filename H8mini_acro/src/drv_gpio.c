#include "gd32f1x0.h"
#include "drv_gpio.h"

void gpio_init(void)
{
// clocks on to all ports		
    RCC_AHBPeriphClock_Enable(RCC_AHBPERIPH_GPIOA|RCC_AHBPERIPH_GPIOB|RCC_AHBPERIPH_GPIOF,ENABLE);
	 // RCC_AHBPeriphClock_Enable(RCC_AHBPERIPH_GPIOB,ENABLE); 
	 // RCC_AHBPeriphClock_Enable(RCC_AHBPERIPH_GPIOF,ENABLE);

	
    GPIO_InitPara GPIO_InitStructure;
	
// common settings to set ports
	  GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
    GPIO_InitStructure.GPIO_OType = GPIO_OTYPE_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_NOPULL;

// port F	
    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_0|GPIO_PIN_1; 
    GPIO_Init(GPIOF,&GPIO_InitStructure);

// port B	
    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_0;
    GPIO_Init(GPIOB,&GPIO_InitStructure);

// port A 
// also used for serial out
#ifndef SERIAL	
    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_2;
    GPIO_Init(GPIOA,&GPIO_InitStructure);
#endif		
}


