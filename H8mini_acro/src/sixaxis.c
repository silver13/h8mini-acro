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
#include "drv_softi2c.h"
#include "util.h"
#include "config.h"
#include "led.h"
#include "drv_serial.h"

#include "drv_i2c.h"

#include <math.h>

#define UNK_INVENSENSE_ADDRESS 0x68


#if (GYROACC_LOW_PASS_FILTER<=6)
#define UNK_INVENSENSE_DLPF_CFG GYROACC_LOW_PASS_FILTER
#else
#define UNK_INVENSENSE_DLPF_CFG   6
#endif



void sixaxis_init( void)
{
// gyro soft reset
// softi2c_write(UNK_INVENSENSE_ADDRESS, 107, 128 );
	
 i2c_writereg( 107 , 128);	
	 
 delay(40000);
	// gyro scale 2000 deg (FS =3)
// softi2c_write(UNK_INVENSENSE_ADDRESS, 27, 24 );

 i2c_writereg( 27 , 24);	
	
//	softi2c_write(UNK_INVENSENSE_ADDRESS, 28, B00001000 ); // acc scale
	
	i2c_writereg( 28 , B00001000);	
	
	// Gyro and acc DLPF low pass filter
 //softi2c_write(UNK_INVENSENSE_ADDRESS, 26, UNK_INVENSENSE_DLPF_CFG ); 

i2c_writereg( 26 , UNK_INVENSENSE_DLPF_CFG);		
}


int sixaxis_check( void)
{
	// read "who am I" register
// return (0x78==softi2c_read( UNK_INVENSENSE_ADDRESS , 117 ) ); 	
	return (0x78==i2c_readreg( 117 ));
}



float accel[3];
float gyro[3];

float gyrocal[3];


void gyro_read( void)
{
int data[6];
	
//softi2c_readdata( 0x68 , 67 , data , 6 );

	
 i2c_readdata( 67 , data, 6 );
	
gyro[1] = (int16_t) ((data[0]<<8) + data[1]);
gyro[0] = (int16_t) ((data[2]<<8) + data[3]);
gyro[2] = (int16_t) ((data[4]<<8) + data[5]);


gyro[0] = gyro[0] - gyrocal[0];
gyro[1] = gyro[1] - gyrocal[1];
gyro[2] = gyro[2] - gyrocal[2];

//printf("gyro0 %f "   , gyro[0]);
//printf("gyro1 %f "   , gyro[1]);
//printf("gyro2 %f "   , gyro[2]);
//printf("\n");

gyro[0] = - gyro[0];
gyro[2] = - gyro[2];


for ( int i = 0 ; i < 3; i++)
{
	gyro[i] = gyro[i] *  0.061035156 * 0.017453292 ;
}


}


int count = 0;

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
		
		//softi2c_readdata( 0x68 , 67 , data, 6 );

 i2c_readdata( 67 , data, 6 );
		
		gyro[0] = (int16_t) ((data[2]<<8) + data[3]);
		gyro[1] = (int16_t) ((data[0]<<8) + data[1]);
		gyro[2] = (int16_t) ((data[4]<<8) + data[5]);	
		
if ( (time - timestart)%200000 > 100000) 
{
	ledon(B0101);
	ledoff(B1010);
}
else 
{
	ledon(B1010);
	ledoff(B0101);
}
		 for ( int i = 0 ; i < 3 ; i++)
			{
				if ( fabs(gyro[i]) > 100 ) 
				{
					count = 0;
					timestart = gettime();
				}
				else
				{
					lpf( &gyrocal[i] , gyro[i], lpfcalc( (float) looptime , 0.5 * 1e6) );
					count++;
				}
				
			}
					
		time = gettime();
	}

if ( count < 100 )
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







