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


#include <math.h>
#include "util.h"
#include "drv_time.h"

// calculates the coefficient for lpf filter, times in the same units
float lpfcalc( float sampleperiod , float filtertime)
{
	if ( sampleperiod <= 0 ) return 0;
  if ( filtertime <= 0 ) return 1;
   float ga = expf(-1.0f/( (1.0f/ sampleperiod) * (filtertime) ));
	if (ga > 1) ga = 1;
	return ga;
}


float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{

return ((x - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min;

}


void lpf( float *out, float in , float coeff)
{
	*out = ( *out )* coeff + in * ( 1-coeff); 
}


void limitf ( float *input , const float limit)
{
	if (*input > limit) *input = limit;
	if (*input < - limit) *input = - limit;		
}

float rcexpo ( float in , float exp )
{
	if ( exp > 1 ) exp = 1;
	if ( exp < -1 ) exp = -1;
	float ans = in*in*in * exp + in * ( 1 - exp );
	limitf( &ans , 1.0);
	return ans;
}


// timing routines for debugging
static unsigned long timestart;
unsigned long timeend;

// timestart
void TS( void)
{
	timestart = gettime(); 
}
// timeend
void TE( void)
{
	timeend =( gettime() - timestart );	
}



float fastsin( float x )
{
 //always wrap input angle to -PI..PI
while (x < -3.14159265)
    x += 6.28318531;

while (x >  3.14159265)
    x -= 6.28318531;
float sin1;

//compute sine
if (x < 0)
   sin1 = (1.27323954 + .405284735 * x) *x;
else
   sin1 = (1.27323954 - .405284735 * x) *x;


return sin1; 
    
} 


float fastcos( float x )
{
 x += 1.57079632;
	return fastsin(x);
}

