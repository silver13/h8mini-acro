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


#include <inttypes.h>
#include "binary.h"
#include "sixaxis.h"
#include "drv_time.h"
//#include "drv_softi2c.h"
#include "util.h"
#include "config.h"
#include "led.h"

#include "drv_serial.h"

#include "drv_i2c.h"

#include <math.h>


#ifdef DEBUG
int gyroid;
#endif

void sixaxis_init( void)
{
// gyro soft reset
	
 i2c_writereg( 107 , 128);	
	 
 delay(40000);
	
// clear sleep bit on old type gyro (mpu-6050)
i2c_writereg( 107 , 0);
	
// gyro scale 2000 deg (FS =3)
i2c_writereg( 27 , 24);	
	
// Gyro DLPF low pass filter
i2c_writereg( 26 , GYRO_LOW_PASS_FILTER);	
	
}


int sixaxis_check( void)
{
	// read "who am I" register
	int id = i2c_readreg( 117 );
	// new board returns 78h (unknown gyro maybe mpu-6500 compatible) marked m681
	// old board returns 68h (mpu - 6050)
	// a new (rare) gyro marked m540 returns 7Dh
	#ifdef DEBUG
	gyroid = id;
	#endif
	#ifdef DISABLE_GYRO_CHECK
	return 1;
	#endif
	return (0x78==id||0x68==id||0x7d==id );
}



float accel[3];
float gyro[3];

float gyrocal[3];


void gyro_read( void)
{
int data[6];

 i2c_readdata( 67 , data, 6 );
	
gyro[1] = (int16_t) ((data[0]<<8) + data[1]);
gyro[0] = (int16_t) ((data[2]<<8) + data[3]);
gyro[2] = (int16_t) ((data[4]<<8) + data[5]);


gyro[0] = gyro[0] - gyrocal[0];
gyro[1] = gyro[1] - gyrocal[1];
gyro[2] = gyro[2] - gyrocal[2];

gyro[0] = - gyro[0];
gyro[2] = - gyro[2];


for ( int i = 0 ; i < 3; i++)
{
	gyro[i] = gyro[i] *  0.061035156f * 0.017453292f ;
}


}
 

void gyro_cal(void)
{
int data[6];
	
unsigned long time = gettime();
unsigned long timestart = time;
unsigned long timemax = time;
unsigned long lastlooptime;

	
// 2 and 15 seconds
while ( time - timestart < 2e6  &&  time - timemax < 15e6 )
	{	
		
		unsigned long looptime; 
		looptime = time - lastlooptime;
		lastlooptime = time;
		if ( looptime == 0 ) looptime = 1;
	

		i2c_readdata( 67 , data, 6 );
		
		gyro[0] = (int16_t) ((data[2]<<8) + data[3]);
		gyro[1] = (int16_t) ((data[0]<<8) + data[1]);
		gyro[2] = (int16_t) ((data[4]<<8) + data[5]);	
		
if ( (time - timestart)%200000 > 100000) 
{
	ledon(B00000101);
	ledoff(B00001010);
}
else 
{
	ledon(B00001010);
	ledoff(B00000101);
}
		 for ( int i = 0 ; i < 3 ; i++)
			{
				if ( fabs(gyro[i]) > 200 ) 
				{
					// restart procedure due to movement
					timestart = gettime();
				}
				else
				{
					// one eighth of calibration time
					lpf( &gyrocal[i] , gyro[i], lpfcalc( (float) looptime , (2e6/8) ) ); 
				}
				
			}

		while(gettime() - time < 1000) delay(1);
		
		time = gettime();
	}

if ( time - timestart < 2e6 )
{
	for ( int i = 0 ; i < 3; i++)
	{
		gyrocal[i] = 0;
	}
}
	
#ifdef SERIAL	
printf("gyro calibration  %f %f %f \n "   , gyrocal[0] , gyrocal[1] , gyrocal[2]);
#endif
	
}







