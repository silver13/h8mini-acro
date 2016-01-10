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

//#include "drv_spi.h"
#include "drv_softi2c.h"

//#define i2cdebug

void delay(int);

#define _delay  //delay(1)
#define _delay2 //delay(1)

	#ifdef i2cdebug
	int debug = 1;   		// prints error info, set in setup()
	#endif
	
	int error1;
	int sda;
	int scl;	
	
	void _sendstart(void);
	void _sendstop(void);
	void sdalow(void);
	void sdahigh(void);
	void scllow(void);
	void sclhigh(void);
	void _restart(void);	
	uint8_t _readbyte( uint8_t); 
	uint8_t _sendbyte( uint8_t);
	int _readsda(void);
	
	int sdaout = 0;
void setoutput(void);

////////////////////////////////
/////////I2C Routines//////////


 void sdalow()
{
	if(!sdaout) setoutput();
	
//	GPIO_WriteBit(GPIOB, GPIO_PIN_7, Bit_RESET);
  GPIOB->BCR = GPIO_PIN_7;
  sda=0;
	_delay;
}


  void sdahigh()
{
	if(!sdaout) setoutput();
	//GPIO_WriteBit(GPIOB, GPIO_PIN_7, Bit_SET);
   GPIOB->BOR = GPIO_PIN_7;
	_delay;
  sda = 1; 
}


  void scllow()
{
// GPIO_WriteBit(GPIOB, GPIO_PIN_6, Bit_RESET);
 GPIOB->BCR = GPIO_PIN_6;
	_delay;
 scl = 0;
}

  void sclhigh()
{
 //GPIO_WriteBit(GPIOB, GPIO_PIN_6, Bit_SET);
 GPIOB->BOR = GPIO_PIN_6;
	_delay;
 scl = 1; 
}

  void sclhighlow()
{
 //GPIO_WriteBit(GPIOB, GPIO_PIN_6, Bit_SET);
 GPIOB->BOR = GPIO_PIN_6;
	_delay;
 GPIOB->BCR = GPIO_PIN_6;
_delay;
 scl = 0;
}

void setinput()
{
	sdaout = 0;
	uint32_t pin = 0x07;
  GPIOB->CTLR  &= ~(GPIO_CTLR_CTLR0 << (pin * 2));
  GPIOB->CTLR |= (((uint32_t)GPIO_MODE_IN) << (pin * 2));    
	_delay2;
 }

void setoutput()
{
	sdaout = 1;
	uint32_t pin = 0x07;
	GPIOB->OMODE &= ~((GPIO_OMODE_OM0) << ((uint16_t)pin));
	GPIOB->OMODE |= (uint16_t)(((uint16_t) GPIO_OTYPE_PP) << ((uint16_t)pin));
	GPIOB->CTLR  &= ~(GPIO_CTLR_CTLR0 << (pin * 2));
	GPIOB->CTLR |= (((uint32_t)GPIO_MODE_OUT) << (pin * 2));
	_delay2;
}

int _readsda()
{
#ifdef i2cdebug
  if (!sda)  printf("_readsda: sda low");
#endif
if ( sdaout) setinput();	
return (Bit_SET == GPIO_ReadInputBit( GPIOB, GPIO_PIN_7) ); 
}


void _sendstart()
{
 if (scl == 0) 
 {
 #ifdef i2cdebug
   printf("_sendstart: scl low"); 
 #endif
  sclhigh();
 } 
 if (sda == 1)
{
  if(!_readsda())
	{
	#ifdef i2cdebug
	printf("_sendstart: sda pulled low by slave"); 
	#endif
	error1 = 1;
	}
  sdalow();
}
 else {
	  #ifdef i2cdebug
    printf("_sendstart: sda low"); 
	  #endif
       } 
}


void _restart()
{
 #ifdef i2cdebug 
 if (scl == 1) printf("_restart: scl high"); 
 #endif
 if (sda == 0) 
 {
   sdahigh();
 }
 sclhigh();
 sdalow();
}

void _sendstop()
{
  
  if (sda == 1) 
  {
    if (!scl) sdalow(); else 
		{
		#ifdef i2cdebug
		printf("stop: error");
		#endif
		}
  }
  if (scl == 0) sclhigh();
  else {
		#ifdef i2cdebug
		printf("stop: scl high");
		#endif
		}
  sdahigh();
 
}



uint8_t _sendbyte( uint8_t value )
{
int i;
 if (scl == 1) 
 {
  scllow();
 }
 
 for ( i = 7; i >= 0 ;i--)
 {
 if ((value >> i)&1) 
 {
  sdahigh();
 }
 else 
 {
  sdalow();
 }
 //sclhigh();
 //scllow();
 sclhighlow();
 }
 
 if (!sda) sdahigh(); // release the line
 //get ack

 sclhigh();
// skip ack since it is not used here
 uint8_t ack;// = _readsda();

  if (ack)
	{
	#ifdef i2cdebug
//	if (debug) Serial.println("NOT RECEIVED"); 
	#endif
	}
 scllow();
return ack; 
}

uint8_t _readbyte(uint8_t ack)  //ACK 1 single byte ACK 0 multiple bytes
{
 uint8_t data=0;
 if (scl == 1)
	{
	error1 = 1;
	#ifdef i2cdebug
	printf("read: scl high");
	#endif
	}
 if ( sda == 0) 
 {
   sdahigh();
 }
 if(!sdaout) setoutput();
 int i;
 for( i = 7; i>=0;i--)
 {
  sclhigh(); 
 if (_readsda() ) data|=(1<<i);
  scllow();
 }

if (ack)  
{
  sdahigh();
} 
else 
{
  sdalow();
}
  sclhigh();
  scllow();
if (sda) sdalow(); 

return data;
}


uint8_t softi2c_write( uint8_t device_address , uint8_t address,uint8_t value)
{
 _sendstart();
 _sendbyte((device_address<<1));
 _sendbyte(address);
  uint8_t ack = _sendbyte(value);
  _sendstop();
  return ack;
}


uint8_t softi2c_read(uint8_t device_address , uint8_t register_address)  
{
 _sendstart();
 _sendbyte((device_address<<1));
 _sendbyte(register_address);
 _restart(); 
 _sendbyte((device_address<<1) + 1);
 uint8_t x = _readbyte(1); 
 _sendstop();
 return x; 
}


void softi2c_writedata(uint8_t device_address ,uint8_t register_address , int *data, int size ) 
{
	int index = 0;
 _sendstart();
 _sendbyte(device_address<<1);
 _sendbyte(register_address);
  
  _sendstop();

	while(index<size)
	{
	_sendbyte(data[index]);
	index++;	
	}
 _sendstop();

}


void softi2c_readdata(uint8_t device_address ,uint8_t register_address , int *data, int size ) 
{
	int index = 0;
 _sendstart();
 _sendbyte(device_address<<1);
 _sendbyte(register_address);
 _restart();
 _sendbyte( (device_address<<1) + 1);
	while(index<size-1)
	{
	data[index] = _readbyte(0);
	index++;	
	}
  data[index] = _readbyte(1);
 _sendstop();
 
}


void softi2c_init()
{

  GPIO_InitPara GPIO_InitStructure;

	
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
  GPIO_InitStructure.GPIO_OType = GPIO_OTYPE_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_PULLUP;
	
  GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUT;

  GPIO_Init(GPIOB,&GPIO_InitStructure);

sdaout = 1;
sda = 0;
scl = 0;
	
sdahigh();
sclhigh();
	
}

uint8_t i2c_error()
{
uint8_t errora = error1;
error1 = 0;
return errora;
}



///////////////////////////////END I2C///////
