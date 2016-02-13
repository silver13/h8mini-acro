#include <gd32f1x0.h>

#include "drv_pwm.h"
#include "config.h"

 TIMER_OCInitPara  TIM_OCInitStructure;

// CENTER ALIGNED PWM METHOD

// set in config.h 

//#define PWM_490HZ
//#define PWM_8KHZ_OLD
//#define PWM_8KHZ
//#define PWM_16KHZ
//#define PWM_24KHZ

// 490Khz
#ifdef PWM_490HZ
#define PWMTOP 16383
#define TIMER_PRESCALER 3
#endif


// 8Khz - ch div 3
#ifdef PWM_8KHZ_OLD
#define PWMTOP 1023
#define TIMER_PRESCALER 3
#endif

// 8Khz
#ifdef PWM_8KHZ
#define PWMTOP 3072
#define TIMER_PRESCALER 1
#endif

// 16Khz
#ifdef PWM_16KHZ
#define PWMTOP 1535
#define TIMER_PRESCALER 1
#endif

// 24Khz
#ifdef PWM_24KHZ
#define PWMTOP 1023
#define TIMER_PRESCALER 1
#endif


// 32Khz
#ifdef PWM_32KHZ
#define PWMTOP 767
#define TIMER_PRESCALER 1
#endif

void pwm_init(void)
{
	
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

    GPIO_Init(GPIOA,&GPIO_InitStructure);
		
    GPIO_PinAFConfig(GPIOA,GPIO_PINSOURCE8,GPIO_AF_2);
	  GPIO_PinAFConfig(GPIOA,GPIO_PINSOURCE9,GPIO_AF_2);
		GPIO_PinAFConfig(GPIOA,GPIO_PINSOURCE10,GPIO_AF_2);
	
    TIMER_BaseInitPara TIM_TimeBaseStructure;

    RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_TIMER1,ENABLE);
    RCC_APB1PeriphClock_Enable(RCC_APB1PERIPH_TIMER3,ENABLE);

// TIMER3 for pins A8 A9 A10

    TIM_TimeBaseStructure.TIMER_Prescaler = TIMER_PRESCALER - 1;  //
    TIM_TimeBaseStructure.TIMER_CounterMode = TIMER_COUNTER_CENTER_ALIGNED2;
    TIM_TimeBaseStructure.TIMER_Period = PWMTOP;
    TIM_TimeBaseStructure.TIMER_ClockDivision = TIMER_CDIV_DIV1;
    TIMER_BaseInit(TIMER3,&TIM_TimeBaseStructure);

 //Ch1 , 2 , 3 
    TIM_OCInitStructure.TIMER_OCMode = TIMER_OC_MODE_PWM1;
    TIM_OCInitStructure.TIMER_OCPolarity = TIMER_OC_POLARITY_HIGH;
    TIM_OCInitStructure.TIMER_OutputState = TIMER_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.TIMER_OCIdleState = TIMER_OC_IDLE_STATE_RESET;
		TIM_OCInitStructure.TIMER_Pulse = 0;	

   TIMER_OC1_Init(TIMER1, &TIM_OCInitStructure);
	 TIMER_OC2_Init(TIMER1, &TIM_OCInitStructure);
	 TIMER_OC3_Init(TIMER1, &TIM_OCInitStructure);
	 
   TIMER_OC4_Init(TIMER3, &TIM_OCInitStructure);
		

   TIMER_CARLPreloadConfig(TIMER3,ENABLE);

		
   TIMER_BaseInit(TIMER1,&TIM_TimeBaseStructure);
	
		
   TIMER_CARLPreloadConfig(TIMER1,ENABLE);

	TIMER_OC1_Preload(TIMER1,TIMER_OC_PRELOAD_DISABLE);
	TIMER_OC2_Preload(TIMER1,TIMER_OC_PRELOAD_DISABLE);
	TIMER_OC3_Preload(TIMER1,TIMER_OC_PRELOAD_DISABLE);
	TIMER_OC4_Preload(TIMER3,TIMER_OC_PRELOAD_DISABLE);
	
  TIMER_Enable( TIMER1, ENABLE );
	TIMER_Enable( TIMER3, ENABLE );

	TIMER_CtrlPWMOutputs(TIMER1,ENABLE);
	TIMER_CtrlPWMOutputs(TIMER3,ENABLE);
	
}


#include  <math.h>

void pwm_set( uint8_t number , float pwm)
{
	pwm = pwm * PWMTOP ;
	
	if ( pwm < 0 ) pwm = 0;
  if ( pwm > PWMTOP ) pwm = PWMTOP;
	
	pwm = lroundf(pwm);
	
  switch( number)
	{
		case 0:
		  TIMER1->CHCC1 = (uint32_t) pwm; 
		break;
		
		case 1:
		  TIMER3->CHCC4 = (uint32_t) pwm; 
		break;
		
		case 2:
		  TIMER1->CHCC2 = (uint32_t) pwm; 
		break;
		
		case 3:
		  TIMER1->CHCC3 = (uint32_t) pwm; 
		break;
		
		default:
			// handle error;
			//
		break;	
				
	}
	
}











