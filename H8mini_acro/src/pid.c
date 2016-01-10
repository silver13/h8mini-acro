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


#include <stdbool.h>
#include "pid.h"
#include "util.h"
#include "config.h"

#include "defines.h"


// Kp											ROLL       PITCH     YAW
float pidkp[PIDNUMBER] = { 13.0e-2 , 13.0e-2  , 10e-1 }; 

// Ki											ROLL       PITCH     YAW
float pidki[PIDNUMBER] = { 15e-1  , 15e-1 , 50e-1 };	

// Kd											ROLL       PITCH     YAW
float pidkd[PIDNUMBER] = { 6.8e-1 , 6.8e-1  , 5.0e-1 };	


// output limit			
const float outlimit[PIDNUMBER] = { 0.8 , 0.8 , 0.4 };

// limit of integral term (abs)
const float integrallimit[PIDNUMBER] = { 0.8 , 0.8 , 0.4 };


// this Kp2 is used for a I-PD controller instead of the above  PI-D
// set the top Kp to zero or use a mix of the 2
// there is no need to use this
float pidkp2[PIDNUMBER] = { 0.0e-2 , 0.0e-2 ,  0e-2 };	

float ierror[PIDNUMBER] = { 0 , 0 , 0};	
static float lastrate[PIDNUMBER];
float pidoutput[PIDNUMBER];

extern float error[PIDNUMBER];
static float lasterror[PIDNUMBER];

extern float looptime;
extern float gyro[3];
extern int onground;

extern float looptime;

float timefactor;

void pid_precalc()
{
	timefactor = 0.0032f / looptime;
}

float pid(int x )
{ 

        if (onground) 
				{
           ierror[x] *= 0.8f;
				}
	
				int iwindup = 0;
				if (( pidoutput[x] == outlimit[x] )&& ( error[x] > 0) )
				{
					iwindup = 1;		
				}
				if (( pidoutput[x] == -outlimit[x])&& ( error[x] < 0) )
				{
					iwindup = 1;				
				}
        if ( !iwindup)
				{
				 // midpoint rule instead of rectangular
         ierror[x] = ierror[x] + (error[x] + lasterror[x]) * 0.5f *  pidki[x] * looptime;
				 //ierror[x] = ierror[x] + error[x] *  pidki[x] * looptime; 					
				}
				
				limitf( &ierror[x] , integrallimit[x] );
		
				// P term
          pidoutput[x] = error[x] * pidkp[x] ;
				
				// P2 (direct feedback) term	
				  pidoutput[x] = pidoutput[x] -   gyro[x] *pidkp2[x];
				
				// I term	
					pidoutput[x] += ierror[x];
			
				// D term		  
					pidoutput[x] = pidoutput[x] - (gyro[x] - lastrate[x]) * pidkd[x] * timefactor  ;
				  
				  limitf(  &pidoutput[x] , outlimit[x]);
					
lastrate[x] = gyro[x];	
lasterror[x] = error[x];

return pidoutput[x];		 		
}

