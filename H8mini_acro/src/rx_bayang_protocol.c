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


#include "binary.h"
#include "drv_spi.h"

#include "gd32f1x0.h"
#include "xn297.h"
#include "drv_time.h"
#include <stdio.h>
#include "config.h"

#include "rx_bayang.h"



void rx_init()
{
	// baseband BB_CAL registers
	spi_cson();
	spi_sendbyte(0x3f); 
	spi_sendbyte(0x4c);
	spi_sendbyte(0x84);
	spi_sendbyte(0x6F);
	spi_sendbyte(0x9c);
	spi_sendbyte(0x20);
	spi_csoff();
	
	delay(1000);
	// RF_CAL registers
	spi_cson();
  spi_sendbyte(0x3e); 
	spi_sendbyte(0xc9);
	spi_sendbyte(0x9a);
	spi_sendbyte(0x80);
	spi_sendbyte(0x61);
	spi_sendbyte(0xbb);
	spi_sendbyte(0xab);
	spi_sendbyte(0x9c);
	spi_csoff();
	
	delay(1000);
	// DEMOD_CAL registers
	spi_cson();
  spi_sendbyte(0x39); 
	spi_sendbyte(0x0b);
	spi_sendbyte(0xdf);
	spi_sendbyte(0xc4);
	spi_sendbyte(0xa7);
	spi_sendbyte(0x03);
	spi_csoff();
	delay(1000);

int rxaddress[5] = { 0 , 0 , 0 , 0 , 0  };
xn_writerxaddress( rxaddress);

	xn_writereg( EN_AA , 0 );	// aa disabled
	xn_writereg( EN_RXADDR , 1 ); // pipe 0 only
	xn_writereg( RF_SETUP , 0);
	xn_writereg( RX_PW_P0 , 15 ); // payload size
	xn_writereg( SETUP_RETR , 0 ); // no retransmissions ( redundant?)
	xn_writereg( SETUP_AW , 3 ); // address size (5 bits)
	xn_command( FLUSH_RX);
  xn_writereg( RF_CH , 0 );  // bind on channel 0
  xn_writereg( 0 , B00001111 ); // power up, crc enabled
	
}

int checkpacket()
{
	int status = xn_command(NOP);
	if ( status&(1<<MASK_RX_DR) )
	{	 // rx clear bit
	  xn_writereg( STATUS , (1<<MASK_RX_DR) );
		//RX packet received
		return 1;
	}
	if( (status & B00001110) != B00001110 )
	{
		// rx fifo not empty		
		return 2;	
	}
	
  return 0;
}

void setchannel(int chan)
{
	 xn_writereg(0x25, chan ); // Set channel frequency		
}


float rx[7];
int rxdata[15];

#define CHANOFFSET 512

float packettodata( int *  data)
{
	return ( ( ( data[0]&0x0003) * 256 + data[1] ) - CHANOFFSET ) * 0.001953125 ;	
}


int decodepacket( void)
{
	if ( rxdata[0] == 165 )
	{
		 int sum = 0;
		 for(int i=0; i<14; i++) 
		 {
			sum += rxdata[i];
		 }	
		if ( (sum&0xFF) == rxdata[14] )
		{
			rx[0] = packettodata( &rxdata[4] );
			rx[1] = packettodata( &rxdata[6] );
			rx[2] = packettodata( &rxdata[10] );
		// throttle		
			rx[3] = ( (rxdata[8]&0x0003) * 256 + rxdata[9] ) * 0.000976562;

		// trims are 50% of controls at max		
	// trims are not used because they interfere with dynamic trims feature of devo firmware
			
//			rx[0] = rx[0] + 0.03225 * 0.5 * (float)(((rxdata[4])>>2) - 31);
//			rx[1] = rx[1] + 0.03225 * 0.5 * (float)(((rxdata[6])>>2) - 31);
//			rx[2] = rx[2] + 0.03225 * 0.5 * (float)(((rxdata[10])>>2) - 31);
				
			rx[4] = (rxdata[2] &  0x08)?1:0; // flip channel
			rx[5] = (rxdata[1] == 0xfa)?1:0; // expert mode
			rx[6] = (rxdata[2] &  0x02)?1:0; // headless channel
			return 1;	// valid packet	
		}
	}
return 0; // failed "crc"
}

	int rfchannel[4];
	int rxaddress[5];
	int rxmode = 0;
	int chan = 0;

void nextchannel()
{
	chan++;
	if (chan > 3 ) chan = 0;
	xn_writereg(0x25, rfchannel[chan] );
}


unsigned long lastrxtime;
unsigned long failsafetime;

int failsafe = 0;


void checkrx( void)
{
	int packetreceived =	checkpacket();
	int pass = 0;
		if ( packetreceived ) 
		{ 
			if ( rxmode == 0)
			{	// rx startup , bind mode
				xn_readpayload( rxdata , 15);
		
				if ( rxdata[0] == 164 ) 
				{// bind packet
					rfchannel[0] = rxdata[6];
					rfchannel[1] = rxdata[7];
					rfchannel[2] = rxdata[8];
					rfchannel[3] = rxdata[9];		
					
					rxaddress[0] = rxdata[1];
					rxaddress[1] = rxdata[2];
					rxaddress[2] = rxdata[3];
					rxaddress[3] = rxdata[4];
					rxaddress[4] = rxdata[5];
					rxmode = 123;				
					xn_writerxaddress( rxaddress );
					setchannel( rfchannel[chan]);
				
					#ifdef SERIAL	
					printf( " BIND \n");
					#endif
				}
			}
			else
			{	// normal mode				
				nextchannel();	
				xn_readpayload( rxdata , 15);
				pass = decodepacket();
				lastrxtime = gettime(); 
				if (pass)
				{
					failsafetime = lastrxtime; 
					failsafe = 0;
				}	
			}// end normal rx mode
				
		}// end packet received
		unsigned long time = gettime();
		
    // packet period 12000
		if( time - lastrxtime > 27000 && rxmode != 0)
		{//  channel with no reception	 
		 lastrxtime = time; 		 
		 nextchannel();	
		}
		if( time - failsafetime > FAILSAFETIME )
		{//  failsafe
		  failsafe = 1;
			rx[0] = 0;
			rx[1] = 0;
			rx[2] = 0;
			rx[3] = 0;
		}

}
	








