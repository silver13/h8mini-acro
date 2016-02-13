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
#include <math.h>

#include "pid.h"
#include "config.h"
#include "util.h"
#include "drv_pwm.h"
#include "control.h"
#include "defines.h"
#include "drv_time.h"


extern float rx[7];
extern float gyro[3];
extern int failsafe;
extern float pidoutput[PIDNUMBER];

int onground = 1;
float thrsum;

float error[PIDNUMBER];
float motormap( float input);

float yawangle;

extern float looptime;

extern char auxchange[AUXNUMBER];
extern char aux[AUXNUMBER];

float motorfilter( float motorin ,int number);

#ifdef NOMOTORS
// to maintain timing or it will be optimized away
float tempx[4];
#endif

void control( void)
{

	// hi rates
	float ratemulti;
	float ratemultiyaw;

	if ( aux[RATES] ) 
	{
		ratemulti = HIRATEMULTI;
		ratemultiyaw = HIRATEMULTIYAW;
	}
	else 
	{
		ratemulti = 1.0;
		ratemultiyaw = 1.0;
	}

	
	yawangle = yawangle + gyro[YAW]*looptime;

	if ( auxchange[HEADLESSMODE] )
	{
		yawangle = 0;
	}
	
	if ( aux[HEADLESSMODE] ) 
	{
		float temp = rx[ROLL];
		rx[ROLL] = rx[ROLL] * cosf( yawangle) - rx[PITCH] * sinf(yawangle );
		rx[PITCH] = rx[PITCH] * cosf( yawangle) + temp * sinf(yawangle ) ;
	}
	
	
	error[ROLL] = rx[ROLL] * (float) MAX_RATE * DEGTORAD * ratemulti - gyro[ROLL];
	error[PITCH] = rx[PITCH] * (float) MAX_RATE * DEGTORAD * ratemulti - gyro[PITCH];
	error[YAW] = rx[YAW] * (float) MAX_RATEYAW * DEGTORAD * ratemultiyaw - gyro[YAW];
	
pid_precalc();

	pid(ROLL);
	pid(PITCH);
	pid(YAW);


	
// map throttle so under 10% it is zero	
float	throttle = mapf(rx[3], 0 , 1 , -0.1 , 1 );
if ( throttle < 0   ) throttle = 0;

// turn motors off if throttle is off and pitch / roll sticks are centered
	if ( failsafe || (throttle < 0.001f && (!ENABLESTIX||  (fabs(rx[ROLL]) < 0.5f && fabs(rx[PITCH]) < 0.5f ) ) ) ) 

	{ // motors off
		for ( int i = 0 ; i <= 3 ; i++)
		{
			pwm_set( i , 0 );	
		}	
		onground = 1;
		thrsum = 0;
		#ifdef MOTOR_FILTER		
		// reset the motor filter
		for ( int i = 0 ; i <= 3 ; i++)
					{		
					motorfilter( 0 , i);
					}	
		#endif
	}
	else
	{
		onground = 0;
		float mix[4];	
		
//		pidoutput[2] += motorchange;
#ifdef INVERT_YAW_PID
pidoutput[2] = -pidoutput[2];			
#endif
		
		mix[MOTOR_FR] = throttle - pidoutput[ROLL] - pidoutput[PITCH] + pidoutput[YAW];		// FR
		mix[MOTOR_FL] = throttle + pidoutput[ROLL] - pidoutput[PITCH] - pidoutput[YAW];		// FL	
		mix[MOTOR_BR] = throttle - pidoutput[ROLL] + pidoutput[PITCH] - pidoutput[YAW];		// BR
		mix[MOTOR_BL] = throttle + pidoutput[ROLL] + pidoutput[PITCH] + pidoutput[YAW];		// BL	
		
#ifdef INVERT_YAW_PID
// we invert again cause it's used by the pid internally (for limit)
pidoutput[2] = -pidoutput[2];			
#endif
			
		
		
#ifdef MOTOR_FILTER		
for ( int i = 0 ; i <= 3 ; i++)
			{
			mix[i] = motorfilter(  mix[i] , i);
			}	
#endif
			
		for ( int i = 0 ; i <= 3 ; i++)
		{
		#ifndef NOMOTORS
		pwm_set( i ,motormap( mix[i] ) );
		#else
		tempx[i] = motormap( mix[i] );
		#endif
		}	
	
		
		thrsum = 0;
		for ( int i = 0 ; i <= 3 ; i++)
		{
			if ( mix[i] < 0 ) mix[i] = 0;
			if ( mix[i] > 1 ) mix[i] = 1;
			thrsum+= mix[i];
		}	
		thrsum = thrsum / 4;
		
	}// end motors on
	
}



float motormap( float input)
{ 
	// this is a thrust to pwm function
	//  float 0 to 1 input and output
	// output can go negative slightly
	// measured eachine motors and prop, stock battery
	// a*x^2 + b*x + c
	// a = 0.262 , b = 0.771 , c = -0.0258

if (input > 1.0f) input = 1.0f;
if (input < 0) input = 0;

input = input*input*0.262f  + input*(0.771f);
input += -0.0258f;

return input;   
}


float hann_lastsample[4];
float hann_lastsample2[4];

// hanning 3 sample filter
float motorfilter( float motorin ,int number)
{
 	float ans = motorin*0.25f + hann_lastsample[number] * 0.5f +   hann_lastsample2[number] * 0.25f ;
	
	hann_lastsample2[number] = hann_lastsample[number];
	hann_lastsample[number] = motorin;
	
	return ans;
}




