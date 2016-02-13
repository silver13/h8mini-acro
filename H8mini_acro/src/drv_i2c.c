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



#include "gd32f1x0.h"
#include <stdint.h>
#include <stdio.h>

#include "drv_i2c.h"

#include "drv_time.h"

#define I2CADDRESS 0x68  

#define WAITFORSTOP

void i2c_init( void)
{

 //   RCC_AHBPeriphClock_Enable(RCC_AHBPERIPH_GPIOB,ENABLE);

    RCC_APB1PeriphClock_Enable(RCC_APB1PERIPH_I2C1,ENABLE);
	
	  GPIO_InitPara GPIO_InitStructure;

	
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_MODE_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
  GPIO_InitStructure.GPIO_OType = GPIO_OTYPE_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_PULLUP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	delay(10);
	// the following checks deal with issues arrising from i2c being stopped while the gyro is sending data
	// this happens mainly in debug mode and perhaps at low voltage reset
int i2cfail = 0;
	// sda is set with pullup
	// if sda is low the gyro might have become stuck while waiting for clock(and is sending a "zero")
	if ( Bit_RESET == GPIO_ReadInputBit( GPIOB, GPIO_PIN_7) )
	{
	i2cfail = 1;	
	}
delay(10);
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_7;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_PULLDOWN;
	GPIO_Init(GPIOB,&GPIO_InitStructure);	
	
		delay(10);
	// sda is set with pulldown
	// if sda is high it could be because the gyro is stuck sending data
	if ( Bit_SET == GPIO_ReadInputBit( GPIOB, GPIO_PIN_7) )
	{
	i2cfail = 1;	
	}
	
	if ( i2cfail )
	{
	
	// set sda pullup so it sends an ack
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_7;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_PULLUP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);	
		
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_PULLUP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
  delay(10);
	for ( int i = 0 ; i < 18 ; i++)
		{// send 9 clock pulses on scl to clear any pending byte
		GPIO_WriteBit(GPIOB, GPIO_PIN_6, Bit_RESET);	
			delay(25);
		GPIO_WriteBit(GPIOB, GPIO_PIN_6, Bit_SET);					
			delay(5);
		if ( Bit_RESET != GPIO_ReadInputBit( GPIOB, GPIO_PIN_7) )
			{
				break;
			}
		}
		
	}
	
  // actual i2c setup
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
  GPIO_InitStructure.GPIO_OType = GPIO_OTYPE_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_PULLUP;

  GPIO_Init(GPIOB,&GPIO_InitStructure);
	
    GPIO_PinAFConfig(GPIOB,GPIO_PINSOURCE6,GPIO_AF_1);
    GPIO_PinAFConfig(GPIOB,GPIO_PINSOURCE7,GPIO_AF_1);
		
		I2C_InitPara I2C_InitStructure;
    I2C_InitStructure.I2C_Protocol = I2C_PROTOCOL_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DUTYCYCLE_2;
    I2C_InitStructure.I2C_BitRate = 400000;
    I2C_InitStructure.I2C_AddressingMode = I2C_ADDRESSING_MODE_7BIT;
    I2C_InitStructure.I2C_DeviceAddress = 0x24;
	
    I2C_Init(I2C1, &I2C_InitStructure);
		
	  I2C_Enable(I2C1,ENABLE);
		I2C_Acknowledge_Enable(I2C1,DISABLE);
		I2C_NACKPosition_Enable(I2C1,I2C_NACKPOSITION_CURRENT);
		
}

unsigned int liberror;

#define I2CTIMEOUTCONDITION (i2ccount < 8092)
static unsigned int i2ccount = 0; 


void i2c_sendheader()
{
	i2ccount = 0;
	while(I2C_GetBitState(I2C1, I2C_FLAG_I2CBSY) && I2CTIMEOUTCONDITION ) i2ccount++;
	
  I2C_StartOnBus_Enable(I2C1, ENABLE);
  while(!I2C_StateDetect(I2C1, I2C_PROGRAMMINGMODE_MASTER_SBSEND && I2CTIMEOUTCONDITION ))i2ccount++;
	I2C_GetBitState(I2C1,I2C_FLAG_SBSEND);
  
  I2C_SendData(I2C1, ( I2CADDRESS<<1));
  while(!I2C_StateDetect(I2C1, I2C_PROGRAMMINGMODE_MASTER_TRANSMITTER_ADDSEND) && I2CTIMEOUTCONDITION)i2ccount++;

}

void i2c_writereg( int reg ,int data)
{
 
 i2c_sendheader();

	I2C_SendData(I2C1,reg);
	I2C_SendData(I2C1,data);
		 
	while(I2C_StateDetect(I2C1,I2C_PROGRAMMINGMODE_MASTER_BYTE_TRANSMITTED) == ERROR && I2CTIMEOUTCONDITION)
	{	i2ccount++; }

	I2C_StopOnBus_Enable(I2C1, ENABLE);
#ifdef WAITFORSTOP	
	while(I2C1->CTLR1&0x0200 && I2CTIMEOUTCONDITION) i2ccount++;
#endif
		
	if ( !I2CTIMEOUTCONDITION ) 
	{
		liberror++;
	}
	
}


int i2c_readreg( int reg )
{
	 int data;

    
i2c_sendheader();
		
	I2C_SendData(I2C1,reg);
	while(I2C_StateDetect(I2C1,I2C_PROGRAMMINGMODE_MASTER_BYTE_TRANSMITTED) == ERROR && I2CTIMEOUTCONDITION)
	{
		i2ccount++;
	}
// restart
	I2C_StartOnBus_Enable(I2C1,ENABLE);
	while(!I2C_StateDetect(I2C1, I2C_PROGRAMMINGMODE_MASTER_SBSEND && I2CTIMEOUTCONDITION))
	{
		i2ccount++;
	}
	
// send receiveing address  
	I2C_SendData(I2C1, ( I2CADDRESS<<1) + 1);
	while(!I2C_StateDetect(I2C1, I2C_PROGRAMMINGMODE_MASTER_RECEIVER_ADDSEND) && I2CTIMEOUTCONDITION)
	{
		i2ccount++;
	}

// wait for one byte
	while(!I2C_StateDetect(I2C1,I2C_PROGRAMMINGMODE_MASTER_BYTE_RECEIVED) && I2CTIMEOUTCONDITION)
	{
		i2ccount++;
	}	
	data = I2C_ReceiveData(I2C1);

	I2C_StopOnBus_Enable(I2C1, ENABLE);
	// wait for stop
#ifdef WAITFORSTOP	
  while(I2C1->CTLR1&0x0200 && I2CTIMEOUTCONDITION)i2ccount++;
#endif
	if ( !I2CTIMEOUTCONDITION ) 
	{
		liberror++;
	}
		
	return data;
}


int i2c_readdata( int reg, int *data, int size )
{
	int index = 0;
	int error = 0;
	int i2ccount2 = 0;
	int byteerror = 0;

i2c_sendheader();

    I2C_SendData(I2C1,reg);

	  while(I2C_StateDetect(I2C1,I2C_PROGRAMMINGMODE_MASTER_BYTE_TRANSMITTED) == ERROR && I2CTIMEOUTCONDITION)i2ccount++;
// restart
		I2C_StartOnBus_Enable(I2C1,ENABLE);
   
    while(!I2C_StateDetect(I2C1, I2C_PROGRAMMINGMODE_MASTER_SBSEND) && I2CTIMEOUTCONDITION)
		{
			i2ccount++;
		}
 // read address  
    I2C_SendData(I2C1, ( I2CADDRESS<<1) + 1);

		I2C_Acknowledge_Enable(I2C1, ENABLE);

    while(!I2C_StateDetect(I2C1, I2C_PROGRAMMINGMODE_MASTER_RECEIVER_ADDSEND) && I2CTIMEOUTCONDITION)
		{
			i2ccount++;
		}

		
while ( index < size)
{		
		i2ccount2 = 0;
	  while(!I2C_StateDetect(I2C1,I2C_PROGRAMMINGMODE_MASTER_BYTE_RECEIVED) && ( i2ccount2 < 2048 ))
		{
			i2ccount2++;
		}
		if ( i2ccount2 >= 2048 ) byteerror = 1;
		
    data[index] = I2C_ReceiveData(I2C1);
	  index++;
	  if ( index == size-1 )  I2C_Acknowledge_Enable(I2C1, DISABLE);
}
    I2C_StopOnBus_Enable(I2C1, ENABLE);
// wait for stop
#ifdef WAITFORSTOP	
   while(I2C1->CTLR1&0x0200 && I2CTIMEOUTCONDITION) i2ccount++;
#endif

	if ( !I2CTIMEOUTCONDITION || byteerror) 
	{
		liberror++;
		error = 1;
	}
  I2C_Acknowledge_Enable(I2C1,DISABLE);
	
	return error;
}























