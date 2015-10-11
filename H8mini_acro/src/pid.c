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


#define PIDNUMBER 3
				
// this Kp is used for a normal PID ( PI-D , really )
float pidkp[PIDNUMBER] = { 0.0e-2 , 0.0e-2  , 10e-1 }; 
//  											ROLL       PITCH     YAW
// this Kp2 is used for a I-PD controller instead of the above PID
float pidkp2[PIDNUMBER] = { 13.5e-2 , 13.5e-2 ,  0e-2 };	
// Ki
float pidki[PIDNUMBER] = { 19.5e-1  , 19.5e-1 , 50e-1 };	
// Kd											ROLL       PITCH     YAW
float pidkd[PIDNUMBER] = { 5.8e-1 , 5.8e-1  , 5.0e-1 };	

// limit of integral term (abs)
#define ITERMLIMIT_FLOAT 1.0	
			
const float outlimit[3] = { 1.0 , 1.0 , 0.4 };

float ierror[PIDNUMBER] = { 0 , 0 , 0};	
float lastrate[PIDNUMBER];
float pidoutput[3];

extern float error[3];
extern float looptime;
extern float gyro[3];
extern int onground;

float pid(int x )
{    
	
        if (onground) 
				{
           ierror[x] *= 0.8;
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
         ierror[x] = ierror[x] + error[x] *  pidki[x] * looptime; 
				}
				
       //  ierror[x] = ierror[x] + error[x] *  pidki[x] * looptime; 
						
		if ( x != 2)
		{	
         if ( ierror[x]  > ITERMLIMIT_FLOAT) ierror[x] = ITERMLIMIT_FLOAT;
				 if ( ierror[x]  < -ITERMLIMIT_FLOAT) ierror[x] = -ITERMLIMIT_FLOAT;
		}else
		{
			limitf( &ierror[x] , 0.2 );
		}
		
				// P term
          pidoutput[x] = error[x] * pidkp[x] ;
			
				// I term	
					pidoutput[x] += ierror[x]  ;
					
				// P2 (direct feedback) term	
					float p2term =  -  ( gyro[x]) *pidkp2[x];
				
				  pidoutput[x] = pidoutput[x] + p2term; 
				
					// D term
					pidoutput[x] = pidoutput[x] - (gyro[x] - lastrate[x]) * pidkd[x]; 
					
				  limitf(  &pidoutput[x] , outlimit[x]);
					
lastrate[x] = gyro[x];	

return pidoutput[x];		 		
}

