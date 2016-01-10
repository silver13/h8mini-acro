/*
The MIT License (MIT)

Copyright (c) 2015 silverx

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/


// Eachine H8mini acro firmware
// files of this project should be assumed MIT licence unless otherwise noted


#include "gd32f1x0.h"

#include "led.h"
#include "util.h"
#include "sixaxis.h"
#include "drv_adc.h"
#include "drv_time.h"
#include "drv_softi2c.h"
#include "config.h"
#include "drv_pwm.h"
#include "drv_adc.h"
#include "drv_gpio.h"
#include "drv_serial.h"
#include "rx_bayang.h"
#include "drv_spi.h"
#include "control.h"
#include "defines.h"
#include "drv_i2c.h"

#include "binary.h"

#include <inttypes.h>

// hal
void clk_init(void);

float looptime;

void failloop( int val);

int main(void)
{
	clk_init();
	
  gpio_init();

#ifdef SERIAL	
	serial_init();
#endif

	i2c_init();
	
	spi_init();
	
	pwm_init();

	pwm_set( MOTOR_FL , 0);   // FL
	pwm_set( MOTOR_FR , 0);	 
	pwm_set( MOTOR_BL , 0);   // BL
	pwm_set( MOTOR_BR , 0);   // FR

  time_init();
	

#ifdef SERIAL	
	printf( "\n clock source:" );
	#endif
	if (  RCC_GetCK_SYSSource() == 8)  
	{
		#ifdef SERIAL	
		printf( " PLL \n" );
		#endif
	}
	else 
	{
	#ifdef SERIAL	
	if (  RCC_GetCK_SYSSource() == 0)  printf( " HSI \n" );
	else  printf( " OTHER \n" );
	#endif
  failloop( 5 );
	}

	sixaxis_init();
	
	if ( sixaxis_check() ) 
	{
		#ifdef SERIAL	
		printf( " MPU found \n" );
		#endif
	}
	else 
	{
		#ifdef SERIAL	
		printf( "ERROR: MPU NOT FOUND \n" );	
		#endif
		failloop(4);
	}
	
	adc_init();
	
	rx_init();
	
int count = 0;
float vbattfilt = 0.0;
	
while ( count < 64 )
{
	vbattfilt += adc_read(1);
	count++;
}
 vbattfilt = vbattfilt/64;	

#ifdef SERIAL	
		printf( "Vbatt %2.2f \n", vbattfilt );
#ifdef NOMOTORS
    printf( "NO MOTORS\n" );
#warning "NO MOTORS"
#endif
#endif
	
#ifdef STOP_LOWBATTERY
// infinite loop
if ( vbattfilt < (float) STOP_LOWBATTERY_TRESH) failloop(2);
#endif

	gyro_cal();

extern unsigned int liberror;
if ( liberror ) 
{
	  #ifdef SERIAL	
		printf( "ERROR: I2C \n" );	
		#endif
		failloop(7);
}

 static unsigned lastlooptime; 
 lastlooptime = gettime();
 extern int rxmode;
 extern int failsafe;

 float thrfilt;

//
//
// 		MAIN LOOP
//
//

#ifdef DEBUG
static float timefilt;
#endif

	while(1)
	{
		// gettime() needs to be called at least once per second 
		unsigned long time = gettime();
		looptime = ((uint32_t)( time - lastlooptime));
		if ( looptime <= 0 ) looptime = 1;
		looptime = looptime * 1e-6f;
		if ( looptime > 0.02f ) // max loop 20ms or else...
		{
			failloop( 3);	
			//endless loop			
		}
		#ifdef DEBUG
		lpf ( &timefilt , looptime, 0.998 );
		#endif
		lastlooptime = time;
		
		if ( liberror > 20) 
		{
			failloop(8);
			// endless loop
		}
		
		checkrx();
		
		gyro_read();
		
		control();
		
// battery low logic
		static int lowbatt = 0;
		float hyst;
		float battadc = adc_read(1);

		// average of all 4 motor thrusts
		// should be proportional with battery current			
		extern float thrsum; // from control.c
		// filter motorpwm so it has the same delay as the filtered voltage
		// ( or they can use a single filter)		
		lpf ( &thrfilt , thrsum , 0.9968f);	// 0.5 sec at 1.6ms loop time	
		
		lpf ( &vbattfilt , battadc , 0.9968f);		

		if ( lowbatt ) hyst = HYST;
		else hyst = 0.0f;
		
		if ( vbattfilt + (float) VDROP_FACTOR * thrfilt <(float) VBATTLOW + hyst ) lowbatt = 1;
		else lowbatt = 0;
		
// led flash logic		
		if ( rxmode == 0)
		{// bind mode
		ledflash ( 100000+ 500000*(lowbatt) , 12);
		}else
		{// non bind
		if ( failsafe) 
		{
		if ( lowbatt )
				ledflash ( 500000 , 8);
		else
				ledflash ( 500000, 15);	
			
		}
			else
			{					
			if ( lowbatt) 
				 ledflash ( 500000 , 8);	
			else ledon( 255);	
			} 		
		}
		
	}// end loop
	

}

// 2 - low battery at powerup - if enabled by config
// 3 - loop time issue
// 4 - Gyro not found
// 5 - clock , intterrupts , systick
// 7 - i2c error 
// 8 - i2c error main loop

void failloop( int val)
{
	for ( int i = 0 ; i <= 3 ; i++)
	{
		pwm_set( i ,0 );
	}	

	while(1)
	{
		for ( int i = 0 ; i < val; i++)
		{
		 ledon( 255);		
		 delay(400000);
		 ledoff( 255);	
		 delay(400000);			
		}
		delay(1600000);
	}	
	
}


void HardFault_Handler(void)
{
	failloop(5);
}
void MemManage_Handler(void) 
{
	failloop(5);
}
void BusFault_Handler(void) 
{
	failloop(5);
}
void UsageFault_Handler(void) 
{
	failloop(5);
}





