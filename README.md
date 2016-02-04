# Eachine H8 mini acro firmware 

This is an acro firmware for the Eachine H8mini quadcopter.

**Do not flash the H8 firmware to the H101**

**Do not flash the H8 firmware to the H8S**


H8 mini hardware:
 * GigaDevice GD32F130G6 cortex M3 32k flash 48Mhz CPU
 * Invensense gyro + accelerometer, MPU-6050 compatible mostly ( responds 0x78 to who am I)
 * actually the unknown gyro is more similar to MPU-6500
 * older version board has MPU-6050 gyro, everything else the same
 * XN297 transceiver, not directly nrf24 compatible but emulated by nrf24 in [deviation tx](http://www.deviationtx.com/) and [nrf24_multipro](https://github.com/goebish/nrf24_multipro)

GD32F130 datasheet (hardware): [GD32F130 (hw)](https://app.box.com/s/3zi661iffmit1rwda499wu8vycv03biv)  (software) : [GD32F130 .pdf (soft)](https://app.box.com/s/pehsanvluc40qu8k2036sbjk5ti08y2m)

The firmware needs Keil.GD32F1xx_DFP.1.1.0.pack which adds support for the cpu to Keil (5.15 used).

config.h - rates, other options
pid.c - pids ( tuned for standard H8 mini)

Thanks to goebish, victzh for bayangtoys protocol reverse engineering and nrf emulation.

Firmware thread featuring flashing info: [rcgroups.com](http://www.rcgroups.com/forums/showthread.php?t=2512604)

#####Stock tx:
On the stock tx only the rate (expert) button works. Trims are not functional on the stock tx.

#####Devo tx:
Channels work as intended except the rate/expert channel which is always on. Dynamic trims are not used, and trims should not be required.

Assign the extra channels to the desired functions in config.h. Right now there is only headless and hi/low rates to set if needed. The defaults should be ok too. ( no headless / rates high always)

#####Gyro calibration
Gyro calibration runs automatically after power up, and usually completes within 2-4 seconds. If for some reason the calibration fails to complete, such as if there is movement, it will eventually time out in 15 seconds.

During calibration the leds flash in an "X" like pattern. If movement is detected the flashing stops and only 2 leds illuminate. The flashing resumes when movement stops.

#####Led error codes
In some cases the leds are used to indicate error conditions, and as such they flash a number of times, then a brake occurs, then the pattern repeats. In all such cases the quadcopter will not respond to commands, a power cycle will be required.

The most common of this is 2 flashes = low battery, usually caused by an in-flight reset due to low battery. All other flashes are non user serviceable. The description is in main.c.

#####Led flash patterns
At startup the leds should flash a gyro calibration pattern for 2 - 15 seconds, in a cross like pattern. Movement stops the flashing while it occurs.

Following should be a fast (20 times/sec) flash indicating that the quad is waiting for bind. 

If binding is completed the leds should light up continuously, while if tx connection is lost they will flash a short time a couple of times / second.

Overriding all this patterns except gyro calibration, is the low battery flash which is a slow, equally spaced on and off flash. 

###23.01.16
* gyro bias fix for larger biases that occur sometimes

###11.01.16 changes
* fix for the incorrect clock setting 
* running at 48Mhz now
* added the new gyro to the gyro check
* pid values are a bit different to account for the timing changes

###31.10.15 changes
* fixed loop time interaction with pid d term
* increased rx sensitivity by setting xn297 HCURR_LNA bit
* optimizations 
* pwm frequency in drv_pwm.c

###older changes not shown
