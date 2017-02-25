#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

#include "m_i2c.h"


/*
  I2C routines for mularsky project.

  See https://www.kernel.org/doc/Documentation/i2c/dev-interface for more information.
*/




#define I2C_FILENAME_PATTERN "/dev/i2c-%d"



/*
  dev - number in /dev/i2c-X path
  address - I2C bus address of slave device
*/
int m_i2c_open_slave(int dev, uint8_t address)
{
	char buffer[sizeof (I2C_FILENAME_PATTERN) + 3] = { 0 };
	snprintf(buffer, sizeof (buffer), I2C_FILENAME_PATTERN, dev);

	int fd = open(buffer, O_RDWR);
	if (fd < 0) {
		return -1;
	}
	if (ioctl(fd, I2C_SLAVE, address) < 0) {
		close(fd);
		return -1;
	}

	return fd;
}




/*
  fd     - device file descriptor
  reg    - number of register to read
  buffer - output for read data
  size   - number of bytes to read
*/
int m_i2c_read(int fd, uint8_t reg, uint8_t * buffer, size_t size)
{
	if (write(fd, &reg, 1) != 1) {
		fprintf(stderr, "%s:%d: write@read failed\n", __FILE__, __LINE__);
		return -1;
	}
	if (read(fd, buffer, size) != size) {
		fprintf(stderr, "%s:%d: read failed\n", __FILE__, __LINE__);
		return -1;
	}
	return 0;
}
