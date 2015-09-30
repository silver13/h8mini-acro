#include <gd32f1x0.h>

#include "drv_pwm.h"

 TIMER_OCInitPara  TIM_OCInitStructure;

void pwm_init(void)
{

	//done in GPIO
	
   // RCC_AHBPeriphClock_Enable( RCC_AHBPERIPH_GPIOA | RCC_AHBPERIPH_GPIOB  , ENABLE);
	
    GPIO_InitPara GPIO_InitStructure;

	// timer 3 pin PB1 config ( ch4 )
    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
    GPIO_InitStructure.GPIO_OType = GPIO_OTYPE_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_PULLDOWN;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
		
    GPIO_PinAFConfig(GPIOB,GPIO_PINSOURCE1,GPIO_AF_1);
	
	// GPIO pins PA8 , 9 ,10 setup ( Timer1 ch 1 , 2 , 3)
	  GPIO_InitStructure.GPIO_Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  //  GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF;
  //  GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
  //  GPIO_InitStructure.GPIO_OType = GPIO_OTYPE_PP;
  //  GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_PULLDOWN;
    GPIO_Init(GPIOA,&GPIO_InitStructure);
		
    GPIO_PinAFConfig(GPIOA,GPIO_PINSOURCE8,GPIO_AF_2);
	  GPIO_PinAFConfig(GPIOA,GPIO_PINSOURCE9,GPIO_AF_2);
		GPIO_PinAFConfig(GPIOA,GPIO_PINSOURCE10,GPIO_AF_2);
	
	
	
    TIMER_BaseInitPara TIM_TimeBaseStructure;

    /* TIMERS clock enable */
    RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_TIMER1,ENABLE);
    RCC_APB1PeriphClock_Enable(RCC_APB1PERIPH_TIMER3,ENABLE);

// TIMER3 for pins A8 A9 A10
//  TIMER_DeInit(TIMER3);
    TIM_TimeBaseStructure.TIMER_Prescaler = 0;  //
    TIM_TimeBaseStructure.TIMER_CounterMode = TIMER_COUNTER_UP;
    TIM_TimeBaseStructure.TIMER_Period = 16383;
    TIM_TimeBaseStructure.TIMER_ClockDivision = TIMER_CDIV_DIV1;
    TIMER_BaseInit(TIMER3,&TIM_TimeBaseStructure);

 //Ch1 , 2 , 3 
    TIM_OCInitStructure.TIMER_OCMode = TIMER_OC_MODE_PWM1;
    TIM_OCInitStructure.TIMER_OCPolarity = TIMER_OC_POLARITY_HIGH;
    TIM_OCInitStructure.TIMER_OutputState = TIMER_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.TIMER_OCIdleState = TIMER_OC_IDLE_STATE_RESET;
		
    TIM_OCInitStructure.TIMER_Pulse = 0;
    TIMER_OC4_Init(TIMER3, &TIM_OCInitStructure);
   // TIMER_OC4_Preload(TIMER3,TIMER_OC_PRELOAD_DISABLE);

		TIMER_CtrlPWMOutputs(TIMER3,ENABLE);

    /* Auto-reload preload enable */
    TIMER_CARLPreloadConfig(TIMER3,ENABLE);
    /* TIMER enable counter*/
    TIMER_Enable( TIMER3, ENABLE );
		

 //   TIMER_DeInit(TIMER1);
  //  TIM_TimeBaseStructure.TIMER_Prescaler = 7;  //
 //   TIM_TimeBaseStructure.TIMER_CounterMode = TIMER_COUNTER_UP;
  //  TIM_TimeBaseStructure.TIMER_Period = 32767;
  //  TIM_TimeBaseStructure.TIMER_ClockDivision = TIMER_CDIV_DIV1;
    TIMER_BaseInit(TIMER1,&TIM_TimeBaseStructure);


  //  TIM_OCInitStructure.TIMER_OCMode = TIMER_OC_MODE_PWM1;
  //  TIM_OCInitStructure.TIMER_OCPolarity = TIMER_OC_POLARITY_HIGH;
  //  TIM_OCInitStructure.TIMER_OutputState = TIMER_OUTPUT_STATE_ENABLE;
  //  TIM_OCInitStructure.TIMER_OCIdleState = TIMER_OC_IDLE_STATE_RESET;
		
  //  TIM_OCInitStructure.TIMER_Pulse = 16383;
    TIMER_OC1_Init(TIMER1, &TIM_OCInitStructure);
 //   TIMER_OC1_Preload(TIMER1,TIMER_OC_PRELOAD_DISABLE);
		
	//	TIM_OCInitStructure.TIMER_Pulse = 16383;
    TIMER_OC2_Init(TIMER1, &TIM_OCInitStructure);
 //   TIMER_OC2_Preload(TIMER1,TIMER_OC_PRELOAD_DISABLE);
		
	//	TIM_OCInitStructure.TIMER_Pulse = 16383;
    TIMER_OC3_Init(TIMER1, &TIM_OCInitStructure);
 //   TIMER_OC3_Preload(TIMER1,TIMER_OC_PRELOAD_DISABLE);
		
		TIMER_CtrlPWMOutputs(TIMER1,ENABLE);	
		
    TIMER_CARLPreloadConfig(TIMER1,ENABLE);

	TIMER_OC1_Preload(TIMER1,TIMER_OC_PRELOAD_DISABLE);
	TIMER_OC2_Preload(TIMER1,TIMER_OC_PRELOAD_DISABLE);
	TIMER_OC3_Preload(TIMER1,TIMER_OC_PRELOAD_DISABLE);
	TIMER_OC4_Preload(TIMER3,TIMER_OC_PRELOAD_DISABLE);
	
    TIMER_Enable( TIMER1, ENABLE );
}

void pwm_set( uint8_t number , float pwm)
{
	pwm = pwm * 16383.0;
	
	if ( pwm < 0 ) pwm = 0;
  if ( pwm > 16383 ) pwm = 16383;
	
  TIM_OCInitStructure.TIMER_Pulse = (uint32_t) pwm;
  switch( number)
	{
		case 0:
			TIMER_OC1_Init(TIMER1, &TIM_OCInitStructure);
		break;
		
		case 1:
			TIMER_OC4_Init(TIMER3, &TIM_OCInitStructure);
		break;
		
		case 2:
			TIMER_OC2_Init(TIMER1, &TIM_OCInitStructure);
		break;
		
		case 3:
			TIMER_OC3_Init(TIMER1, &TIM_OCInitStructure);
		break;
		
		default:
			// handle error;
			//
		break;	
				
	}
	
}











