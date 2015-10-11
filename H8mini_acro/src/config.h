

// pids in pid.c


// rate in deg/sec
// for low rates
#define MAX_RATE 180.0
#define MAX_RATEYAW 360.0

// disable inbuilt expo functions
#define DISABLE_EXPO

// use if your tx has no expo function
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

// voltage to start warning
#define VBATTLOW 3.6

// compensation for battery voltage vs throttle drop
// increase if battery low comes on at max throttle
// decrease if battery low warning goes away at high throttle
#define VDROP_FACTOR 0.60

// voltage hysteresys
#define HYST 0.10

// enable serial out on back-left LED
// serial is quite slow 
//#define SERIAL

// gyro filter 3 = 42hz
#define GYROACC_LOW_PASS_FILTER 3

// 
// disable motors for testing
//#define NOMOTORS

























