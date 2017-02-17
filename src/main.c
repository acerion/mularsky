#define _BSD_SOURCE

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <linux/i2c-dev.h>




/*
  Unless otherwise noted, references to chapters or tables refer to BME280 datasheet from Bosh.

  BME280: Final data sheet
  Document revision 1.1
  Document release date May 07 , 2015
  Document number BST-BME280-DS001-10
  Technical reference code(s) 0 273 141 185
*/




/* i2c-specific definitions. */
#define I2C_FILENAME "/dev/i2c-1"
#define BME280_I2C_ADDR 0x77




/* BME280 configuration registers and their contents (chip configuration). */

#define BME280_REG_CHIP_ID         0xD0   /* Read only. */

#define BME280_REG_CTRL_CONFIG     0xF5
#define BME280_SETTING_STBY        0xe0   /* 111x xxxx = 250 ms (table 27). */
#define BME280_SETTING_FILTER      0x10   /* xxx1 00xx = filter coeff 16. */

#define BME280_REG_CTRL_HUM        0xF2
#define BME280_SETTING_HUM_OS      0x05   /* xxxx x101 = 16x oversampling (table 20). */

#define BME280_REG_CTRL_MEAS       0xF4
#define BME280_SETTING_TEMP_OS     0xa0   /* 101x xxxx = 16x oversampling (table 24). */
#define BME280_SETTING_PRESS_OS    0x14   /* xxx1 01xx = 16x oversampling (table 23). */
#define BME280_SETTING_MODE        0x03   /* xxxx xx11 = Normal mode (table 25). */




static int pressure_fd = 0;
static int m_i2c_read(int fd, uint8_t reg, uint8_t * buffer, size_t size);
static void m_bme280_convert_and_store_data(const uint8_t * buffer);
static void m_bme280_convert_and_store_compensation(const uint8_t * buffer);




int main(int argc, char ** argv)
{
	if (argc != 2) {
		fprintf(stderr, "%s <numer of reads>\n", argv[0]);
		return -1;
	}



	/* Open a device. */
	{
		/* https://www.kernel.org/doc/Documentation/i2c/dev-interface */
		pressure_fd = open(I2C_FILENAME, O_RDWR);
		if (pressure_fd < 0) {
			exit(1);
		}
		if (ioctl(pressure_fd, I2C_SLAVE, BME280_I2C_ADDR) < 0) {
			close(pressure_fd);
			exit(1);
		}
	}



	/*
	  Read Chip ID.

	  Chapter 5.4.1 Register 0xD0 "id".
	  "This number can be read as soon as the device finished the power-on-reset."
	*/
	{
		uint8_t id = 0;
		int rv = m_i2c_read(pressure_fd, BME280_REG_CHIP_ID, &id, 1);
		fprintf(stderr, "%s:%d: rv = %d, chip id = 0x%02x\n", __FILE__, __LINE__, rv, id);
	}



	/* Read compensation data. */
	{
		uint8_t block_start = 0;
		uint8_t buffer[24 + 1 + 7] = { 0 };
		int rv = 0;

		block_start = 0x88;
		buffer[0] = block_start;
		rv = m_i2c_read(pressure_fd, block_start, buffer + 0, 24); /* Read 24 bytes, store them at the beginning of buffer. */
		if (rv == -1) {
			fprintf(stderr, "%s:%d: read compensation 1 failed\n", __FILE__, __LINE__);
			return -1;
		}


		block_start = 0xa1;
		buffer[0 + 24] = block_start;
		rv = m_i2c_read(pressure_fd, block_start, buffer + 0 + 24, 1); /* Read 1 byte, store it in cell #25 of buffer. */
		if (rv == -1) {
			fprintf(stderr, "%s:%d: read compensation 2 failed\n", __FILE__, __LINE__);
			return -1;
		}


		block_start = 0xe1;
		buffer[0 + 24 + 1] = block_start;
		rv = m_i2c_read(pressure_fd, block_start, buffer + 0 + 24 + 1, 7); /* Read 7 bytes, store them in cells #26-#32. */
		if (rv == -1) {
			fprintf(stderr, "%s:%d: read compensation 3 failed\n", __FILE__, __LINE__);
			return -1;
		}

		m_bme280_convert_and_store_compensation(buffer);
	}



	/* Configuration. */
	{
		uint8_t buffer[2];

		/*
		  Chapter 5.4.6 Register 0xF5 "config"
		  "In sleep mode writes are not ignored."

		  Per chapter 3.3, after startup the chip is in sleep
		  mode, but later the chip may be in normal mode.  So
		  let's first force sleep mode.
		*/
		buffer[0] = BME280_REG_CTRL_MEAS;
		buffer[1] = 0x00;
		if (write(pressure_fd, buffer, 2) != 2) {
			fprintf(stderr, "%s:%d: write config 0 failed\n", __FILE__, __LINE__);
			return -1;
		}



		/*
		  Now we are in sleep mode and ready to configure device and then return to normal mode.

		  Chapter 3.3.4 Normal mode
		  "After setting the measurement and filter options
		  and enabling normal mode, the last measurement
		  results can always be obtained at the data registers
		  without the need of further write accesses."
		*/

		buffer[0] = BME280_REG_CTRL_CONFIG;
		buffer[1] = BME280_SETTING_STBY | BME280_SETTING_FILTER;
		if (write(pressure_fd, buffer, 2) != 2) {
			fprintf(stderr, "%s:%d: write config 1 failed\n", __FILE__, __LINE__);
			return -1;
		}

		buffer[0] = BME280_REG_CTRL_HUM;
		buffer[1] = BME280_SETTING_HUM_OS;
		if (write(pressure_fd, buffer, 2) != 2) {
			fprintf(stderr, "%s:%d: write config 2 failed\n", __FILE__, __LINE__);
			return -1;
		}

		/* Set remainder of config data and go back to normal mode - in one write. */
		buffer[0] = BME280_REG_CTRL_MEAS;
		buffer[1] = BME280_SETTING_TEMP_OS | BME280_SETTING_PRESS_OS | BME280_SETTING_MODE;
		if (write(pressure_fd, buffer, 2) != 2) {
			fprintf(stderr, "%s:%d: write config 3 failed\n", __FILE__, __LINE__);
			return -1;
		}
	}



	/*
	  Read loop.

	  Chapter 3.3.4 Normal mode

	  "After setting the measurement and filter options and
	  enabling normal mode, the last measurement results can
	  always be obtained at the data registers without the need of
	  further write accesses."
	  What does "without the need of further write accesses" mean? I have to give register address, right?
	*/
	{
		const uint8_t block_start = 0xF7;  /* Beginning of data for burst read. */
		const size_t block_size = 8;       /* 0xF7 to 0xFE: pressure, temperature, humidity. */

		uint8_t buffer[block_size];

		for (int i = 0; i < atoi(argv[1]); i++) {
			buffer[0] = block_start;
			int rv = m_i2c_read(pressure_fd, block_start, buffer, block_size);
			if (rv == -1) {
				fprintf(stderr, "%s:%d: read data failed\n", __FILE__, __LINE__);
				return -1;
			}
			fprintf(stderr, "%02x %02x %02x %02x %02x %02x %02x %02x\n",
				buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7]);

			m_bme280_convert_and_store_data(buffer);

			usleep(1000 * 1000);
		}
	}



	close(pressure_fd);
	return 0;
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




/*
  Measurement data is stored in buffer of size 8 bytes.
  The data has been read in burst read of 8 bytes starting from 0xF7.
*/
void m_bme280_convert_and_store_data(const uint8_t * buffer)
{
	/* Chapter 4 Data readout.
	   "The data are read out in an unsigned 20-bit format both
	   for pressure and for temperature and in an unsigned 16-bit
	   format for humidity."
	   I'm using unsigned 32 bits for all three of them for consistency. */

	uint32_t raw_pressure =
		buffer[0] << 12     /* press_msb */
		| buffer[1] << 4    /* press_lsb */
		| buffer[2] >> 4;   /* press_xlsb */

	uint32_t raw_temperature =
		buffer[3] << 12     /* temp_msb */
		| buffer[4] << 4    /* temp_lsb */
		| buffer[5] >> 4;   /* temp_xlsb */

	uint32_t raw_humidity =
		buffer[6] << 8      /* hum_msb */
		| buffer[7];        /* hum_lsb */

	fprintf(stderr, "raw pressure = %u, raw temp = %u, raw humidity = %u\n", raw_pressure, raw_temperature, raw_humidity);

	return;
}




/*
  Compensation data described in
  Chapter 4.2.2 Trimming parameter readout

  The compensation data has been read into one continuous table of size 33 bytes.
*/
void m_bme280_convert_and_store_compensation(const uint8_t * buffer)
{
	/* The compensation data should be stored in bme280 data
	   structure for later use by compensation functions. */

	for (int i = 0; i < 33; i++) {
		fprintf(stderr, "compensation data byte %02d: 0x%02x\n", i, *(buffer + i));
	}

	return;
}
