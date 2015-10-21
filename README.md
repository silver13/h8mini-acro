# Eachine H8 mini acro firmware 

This is an acro firmware for the Eachine H8mini quadcopter.

Initial release may contain bugs and "unintended features".

H8 mini hardware:
 * GigaDevice GD32F130G6 cortex M3 32k flash 72Mhz CPU
 * Invensense gyro + accelerometer, MPU-6050 compatible mostly ( responds 0x78 to who am I)
 * XN297 transceiver, not directly nrf24 compatible but emulated by nrf24 in [deviation tx](http://www.deviationtx.com/) and [nrf24_multipro](https://github.com/goebish/nrf24_multipro)

Cpu datasheet: [Pdf](https://app.box.com/s/3zi661iffmit1rwda499wu8vycv03biv) Cpu Documentation: [Pdf](https://app.box.com/s/pehsanvluc40qu8k2036sbjk5ti08y2m)

The firmware needs Keil.GD32F1xx_DFP.1.1.0.pack which adds support for the cpu to Keil (5.15 used).


Thanks to goebish, victzh for bayang protocol reverse engineering and nrf emulation.

Firmware thread featuring flashing info: [rcgroups.com](http://www.rcgroups.com/forums/showthread.php?t=2512604)

#22.10.15 changes
* fixed rx bug
* fixed expo bug
* changed pids (again) for faster response
* added headless mode

#11.10.15 changes
* a lot of changes this time
* hardware i2c
* better pid tuneings
* measured motor curves
* improved rx code

#04.10.15 changes

* disabled quad side expo functions by default ( should enable in config if required)
* disabled trims since they are not needed in acro mode and they interfere with dynamic trims feature from devo
* minor soft_spi optimization
