# Eachine H8 mini acro firmware 

This is an acro firmware for the Eachine H8mini quadcopter.

**Do not flash the H8 firmware to the H101**

**Do not flash the H8 firmware to the H8S**


H8 mini hardware:
 * GigaDevice GD32F130G6 cortex M3 32k flash 72Mhz CPU
 * Invensense gyro + accelerometer, MPU-6050 compatible mostly ( responds 0x78 to who am I)
 * actually the unknown gyro is more similar to MPU-6500
 * older version board has MPU-6050 gyro, everything else the same
 * XN297 transceiver, not directly nrf24 compatible but emulated by nrf24 in [deviation tx](http://www.deviationtx.com/) and [nrf24_multipro](https://github.com/goebish/nrf24_multipro)

GD32F130 datasheet (hardware): [GD32F130 (hw)](https://app.box.com/s/3zi661iffmit1rwda499wu8vycv03biv) GD32F130 Documentation (software) : [GD32F130 .pdf (soft)](https://app.box.com/s/pehsanvluc40qu8k2036sbjk5ti08y2m)

The firmware needs Keil.GD32F1xx_DFP.1.1.0.pack which adds support for the cpu to Keil (5.15 used).

config.h - rates, other options
pid.c - pids ( tuned for standard H8 mini)

Thanks to goebish, victzh for bayang protocol reverse engineering and nrf emulation.

Firmware thread featuring flashing info: [rcgroups.com](http://www.rcgroups.com/forums/showthread.php?t=2512604)

###31.10.15 changes
* fixed loop time interaction with pid d term
* increased rx sensitivity by setting xn297 HCURR_LNA bit
* optimizations 
* pwm frequency in drv_pwm.c

###older changes not shown
