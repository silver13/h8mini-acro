#include <inttypes.h>


void softi2c_init(void);
void softi2c_readdata(uint8_t device_address ,uint8_t register_address , int *data, int size );
void softi2c_writedata(uint8_t device_address ,uint8_t register_address , int *data, int size );

uint8_t softi2c_read(uint8_t device_address , uint8_t register_address);
uint8_t softi2c_write( uint8_t device_address , uint8_t address,uint8_t value);








