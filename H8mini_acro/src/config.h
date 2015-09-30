

// pids in pid.c


// rate in deg/sec
#define MAX_RATE 180.0
#define MAX_RATEYAW 360.0

// -1 to 1 , 0 = no expo
#define EXPO_XY 0.4
#define EXPO_YAW 0.0

// multiplier for high rates
#define HIRATEMULTI 1.5
#define HIRATEMULTIYAW 1.5


// failsafe time in uS
#define FAILSAFETIME 1000000  // one second


// battery saver
// do not start software if battery is too low
// flashes 2 times repeatedly at startup
//#define STOP_LOWBATTERY
#define STOP_LOWBATTERY_TRESH 3.3

// voltage to start warning
#define VBATTLOW 3.5

// compensation for battery voltage vs throttle drop
// increase if battery low comes on at max throttle
// decrease if battery low warning goes away at high throttle
#define VDROP_FACTOR 0.70

// voltage hysteresys
#define HYST 0.10

// enable serial out on back-left LED
// serial is quite slow atm due to blocking
//#define SERIAL

// gyro filter 3 = 42hz
#define GYROACC_LOW_PASS_FILTER 3



























