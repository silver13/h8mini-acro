
#include "defines.h"

// pids in pid.c

// rate in deg/sec
// for low rates
#define MAX_RATE 180.0
#define MAX_RATEYAW 360.0

// disable inbuilt expo functions
#define DISABLE_EXPO

// use if your tx has no expo function
// also comment out DISABLE_EXPO to use
// -1 to 1 , 0 = no exp
// positive = less sensitive near center 
#define EXPO_XY 0.3
#define EXPO_YAW 0.0


// multiplier for high rates
// devo/module uses high rates only
#define HIRATEMULTI 2.0
#define HIRATEMULTIYAW 2.0


// failsafe time in uS
#define FAILSAFETIME 1000000  // one second


// battery saver
// do not start software if battery is too low
// flashes 2 times repeatedly at startup
#define STOP_LOWBATTERY

// under this voltage the software will not start 
// if STOP_LOWBATTERY is defined above
#define STOP_LOWBATTERY_TRESH 3.3

// voltage too start warning
// volts
#define VBATTLOW 3.5

// compensation for battery voltage vs throttle drop
// increase if battery low comes on at max throttle
// decrease if battery low warning goes away at high throttle
// in volts
#define VDROP_FACTOR 0.60

// voltage hysteresys
// in volts
#define HYST 0.10


// Gyro LPF filter frequency
// gyro filter 0 = 260hz
// gyro filter 1 = 184hz
// gyro filter 2 = 94hz
// gyro filter 3 = 42hz
// 4 , 5, 6
#define GYRO_LOW_PASS_FILTER 3




// channel for headless mode switch
// 0 - flip
// 1 - expert
// 2 - headfree
// 3 - headingreturn
// 4 - on always
// 5 - off always
#define HEADLESSMODE 5

// channel for rates switch
// 0 - flip
// 1 - expert
// 2 - headfree
// 3 - headingreturn
// 4 - on always
// 5 - off always
#define RATES 1



// enable motors if pitch / roll controls off center (at zero throttle)
// possible values: 0 / 1
#define ENABLESTIX 0












// debug / other things
// this should not be usually changed

// disable motors for testing
// #define NOMOTORS


// enable serial out on back-left LED
// serial is quite slow 
// #define SERIAL


// debug things
// #define DEBUG

// disable the check for known gyro that causes the 4 times flash
// #define DISABLE_GYRO_CHECK




#pragma diag_warning 1035 , 177 , 4017
#pragma diag_error 260

























