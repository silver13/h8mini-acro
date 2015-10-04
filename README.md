# Eachine H8 mini acro firmware 

This is a acro firmware for the Eachine H8mini quadcopter.

Initial release may contain bugs and "unintended features".

H8 mini hardware:
 * GigaDevice GD32F130G6 cortex M3 32k flash 72Mhz CPU
 * Invensense gyro + accelerometer, MPU-6050 compatible mostly ( responds 0x78 to who am I)
 * XN297 transceiver, not directly nrf24 compatible but emulated by nrf24 in [deviation tx](http://www.deviationtx.com/) and [nrf24_multipro](https://github.com/goebish/nrf24_multipro)

The firmware needs Keil.GD32F1xx_DFP.1.1.0.pack which adds support for the cpu to Keil (5.15 used).


Thanks to goebish, victzh for bayang protocol reverse engineering and nrf emulation.

Firmware thread featuring flashing info: [rcgroups.com](http://www.rcgroups.com/forums/showthread.php?t=2512604)

#04.10.15 changes

* disabled quad side expo functions by default ( should enable in config if required)
* disabled trims since they are not needed in acro mode and they interfere with dynamic trims feature from devo
* minor soft_spi optimization
