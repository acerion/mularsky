#ifndef H_M_I2C
#define H_M_I2C




#include <stdint.h>




int m_i2c_open_slave(uint8_t address);
int m_i2c_read(int fd, uint8_t reg, uint8_t * buffer, size_t size);




#endif /* #ifndef H_M_I2C */
