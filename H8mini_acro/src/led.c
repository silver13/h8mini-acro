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
#include "drv_time.h"
#include "led.h"

#define LEDALL 15

void ledon( uint8_t val )
{
#ifndef SERIAL	
	if ( val&8) 	GPIO_WriteBit(GPIOA, GPIO_PIN_2, Bit_SET);
#endif
	if ( val&1) GPIO_WriteBit(GPIOF, GPIO_PIN_0, Bit_SET);
	if ( val&2) GPIO_WriteBit(GPIOF, GPIO_PIN_1, Bit_SET);
			
	if (val&4) GPIO_WriteBit(GPIOB, GPIO_PIN_0, Bit_SET);
			
}

void ledoff( uint8_t val )
{
#ifndef SERIAL	
	if ( val&8) 	GPIO_WriteBit(GPIOA, GPIO_PIN_2, Bit_RESET);
#endif
	if ( val&1) GPIO_WriteBit(GPIOF, GPIO_PIN_0, Bit_RESET);
	if ( val&2) GPIO_WriteBit(GPIOF, GPIO_PIN_1, Bit_RESET);	
			
  if (val&4) GPIO_WriteBit(GPIOB, GPIO_PIN_0, Bit_RESET);			
}

void ledset( int val )
{
#ifndef SERIAL	
	if ( val&8) GPIO_WriteBit(GPIOA, GPIO_PIN_2, Bit_SET);
	else GPIO_WriteBit(GPIOA, GPIO_PIN_2, Bit_SET);
#endif
	if ( val&1) GPIO_WriteBit(GPIOF, GPIO_PIN_0, Bit_SET);
	else GPIO_WriteBit(GPIOF, GPIO_PIN_0, Bit_RESET);
	if ( val&2) GPIO_WriteBit(GPIOF, GPIO_PIN_1, Bit_SET);	
	else GPIO_WriteBit(GPIOF, GPIO_PIN_1, Bit_RESET);	 
			
  if (val&4) GPIO_WriteBit(GPIOB, GPIO_PIN_0, Bit_SET);		
	else GPIO_WriteBit(GPIOB, GPIO_PIN_0, Bit_RESET);			
}

void ledflash( uint32_t period , int duty )
{
	if ( gettime() % period > (period*duty)/16 )
	{
		ledon(LEDALL);
	}
	else
	{
		ledoff(LEDALL);
	}
	
	
}












