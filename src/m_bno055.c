#define _BSD_SOURCE /* usleep() */

#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>

#include "m_bno055.h"
#include "m_i2c.h"




int imu_sensor_fd = 0;

extern bool cancel_treads;


struct m_bno055_compensation { int i; };

static FILE * imu_out_fd;
static struct m_bno055_compensation bno055_comp;
static const int imu_ms = 1000; /* [milliseconds] */




#define BNO055_I2C_ADDR 0x77

/* BNO055 configuration registers and their contents (chip configuration). */

#define BNO055_REG_CHIP_ID         0xD0   /* Read only. */

#define BNO055_REG_CTRL_CONFIG     0xF5
#define BNO055_SETTING_STBY        0xe0   /* 111x xxxx = 250 ms (table 27). */
#define BNO055_SETTING_FILTER      0x10   /* xxx1 00xx = filter coeff 16. */

#define BNO055_REG_CTRL_HUM        0xF2
#define BNO055_SETTING_HUM_OS      0x05   /* xxxx x101 = 16x oversampling (table 20). */

#define BNO055_REG_CTRL_MEAS       0xF4
#define BNO055_SETTING_TEMP_OS     0xa0   /* 101x xxxx = 16x oversampling (table 24). */
#define BNO055_SETTING_PRESS_OS    0x14   /* xxx1 01xx = 16x oversampling (table 23). */
#define BNO055_SETTING_MODE        0x03   /* xxxx xx11 = Normal mode (table 25). */




static void m_bno055_convert_and_store_compensation(const uint8_t * buffer, struct m_bno055_compensation * c);
static uint8_t m_bno055_read_chip_id(int fd);
static int m_bno055_configure(int fd);
static int m_bno055_get_compensation_data(int fd, struct m_bno055_compensation * c);
static void m_bno055_convert_and_store_data(const uint8_t * buffer, struct m_bno055_compensation * c);
static int m_bno055_read_loop(int fd, int ms, struct m_bno055_compensation * c);




/*
  Read Chip ID.

  Chapter 5.4.1 Register 0xD0 "id".
  "This number can be read as soon as the device finished the power-on-reset."
*/
uint8_t m_bno055_read_chip_id(int fd)
{
	uint8_t id = 0;
	if (-1 == m_i2c_read(fd, BNO055_REG_CHIP_ID, &id, 1)) {
		fprintf(imu_out_fd, "%s:%d: failed to read imu chip id\n", __FILE__, __LINE__);
		return 0;
	}

	return id;
}




int m_bno055_configure(int fd)
{
#if 0
	uint8_t buffer[2];

	/*
	  Chapter 5.4.6 Register 0xF5 "config"
	  "In sleep mode writes are not ignored."

	  Per chapter 3.3, after startup the chip is in sleep
	  mode, but later the chip may be in normal mode.  So
	  let's first force sleep mode.
	*/
	buffer[0] = BNO055_REG_CTRL_MEAS;
	buffer[1] = 0x00;
	if (write(fd, buffer, 2) != 2) {
		fprintf(imu_out_fd, "%s:%d: write config 0 failed\n", __FILE__, __LINE__);
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

	buffer[0] = BNO055_REG_CTRL_CONFIG;
	buffer[1] = BNO055_SETTING_STBY | BNO055_SETTING_FILTER;
	if (write(fd, buffer, 2) != 2) {
		fprintf(imu_out_fd, "%s:%d: write config 1 failed\n", __FILE__, __LINE__);
		return -1;
	}

	buffer[0] = BNO055_REG_CTRL_HUM;
	buffer[1] = BNO055_SETTING_HUM_OS;
	if (write(fd, buffer, 2) != 2) {
		fprintf(imu_out_fd, "%s:%d: write config 2 failed\n", __FILE__, __LINE__);
			return -1;
	}

	/* Set remainder of config data and go back to normal mode - in one write. */
	buffer[0] = BNO055_REG_CTRL_MEAS;
	buffer[1] = BNO055_SETTING_TEMP_OS | BNO055_SETTING_PRESS_OS | BNO055_SETTING_MODE;
	if (write(fd, buffer, 2) != 2) {
		fprintf(imu_out_fd, "%s:%d: write config 3 failed\n", __FILE__, __LINE__);
		return -1;
	}
#endif

	return 0;
}




int m_bno055_get_compensation_data(int fd, struct m_bno055_compensation * c)
{
#if 0
	uint8_t block_start = 0;
	uint8_t buffer[24 + 1 + 7] = { 0 };
	int rv = 0;

	block_start = 0x88;
	buffer[0] = block_start;
	rv = m_i2c_read(fd, block_start, buffer + 0, 24); /* Read 24 bytes, store them at the beginning of buffer. */
	if (rv == -1) {
		fprintf(imu_out_fd, "%s:%d: read compensation 1 failed\n", __FILE__, __LINE__);
		return -1;
	}


	block_start = 0xa1;
	buffer[0 + 24] = block_start;
	rv = m_i2c_read(fd, block_start, buffer + 0 + 24, 1); /* Read 1 byte, store it in cell #25 of buffer. */
	if (rv == -1) {
		fprintf(imu_out_fd, "%s:%d: read compensation 2 failed\n", __FILE__, __LINE__);
		return -1;
	}


	block_start = 0xe1;
	buffer[0 + 24 + 1] = block_start;
	rv = m_i2c_read(fd, block_start, buffer + 0 + 24 + 1, 7); /* Read 7 bytes, store them in cells #26-#32. */
	if (rv == -1) {
		fprintf(imu_out_fd, "%s:%d: read compensation 3 failed\n", __FILE__, __LINE__);
		return -1;
	}

	m_bno055_convert_and_store_compensation(buffer, c);
#endif
	return 0;
}




void m_bno055_convert_and_store_compensation(const uint8_t * buffer, struct m_bno055_compensation * c)
{
#if 0
	/* The compensation data should be stored in bno055 data
	   structure for later use by compensation functions. */

	fprintf(imu_out_fd, "\n");
	for (int i = 0; i < 33; i++) {
		fprintf(imu_out_fd, "compensation data byte %02d: 0x%02x\n", i, *(buffer + i));
	}

	c->dig_T1 = buffer[0]  | (uint16_t) buffer[1] << 8;
	c->dig_T2 = buffer[2]  | (uint16_t) buffer[3] << 8;
	c->dig_T3 = buffer[4]  | (uint16_t) buffer[5] << 8;

	c->dig_P1 = buffer[6]  | (uint16_t) buffer[7] << 8;
	c->dig_P2 = buffer[8]  | (uint16_t) buffer[9] << 8;
	c->dig_P3 = buffer[10] | (uint16_t) buffer[11] << 8;
	c->dig_P4 = buffer[12] | (uint16_t) buffer[13] << 8;
	c->dig_P5 = buffer[14] | (uint16_t) buffer[15] << 8;
	c->dig_P6 = buffer[16] | (uint16_t) buffer[17] << 8;
	c->dig_P7 = buffer[18] | (uint16_t) buffer[19] << 8;
	c->dig_P8 = buffer[20] | (uint16_t) buffer[21] << 8;
	c->dig_P9 = buffer[22] | (uint16_t) buffer[23] << 8;

	c->dig_H1 = buffer[24];
	c->dig_H2 = buffer[25] | (uint16_t) buffer[26] << 8;
	c->dig_H3 = buffer[27];
	c->dig_H4 = ((uint16_t) buffer[28] << 4) | (0x0f & buffer[29]); /* This is probably messed up. */
	c->dig_H5 = ((0xf0 & buffer[29]) >> 4) | ((uint16_t) buffer[30] << 4); /* This may be messed up as well. */
	c->dig_H6 = buffer[31];


	fprintf(imu_out_fd, "\n");

	/* Printing both hex and decimal to see how the conversion from two's complement looks like. */
	fprintf(imu_out_fd, "comp: T1 = 0x%08x / %u\n",   c->dig_T1, c->dig_T1);
	fprintf(imu_out_fd, "comp: T2 = 0x%08x / %d\n",   c->dig_T2, c->dig_T2);
	fprintf(imu_out_fd, "comp: T3 = 0x%08x / %d\n\n", c->dig_T3, c->dig_T3);

	fprintf(imu_out_fd, "comp: P1 = 0x%08x / %u\n",   c->dig_P1, c->dig_P1);
	fprintf(imu_out_fd, "comp: P2 = 0x%08x / %d\n",   c->dig_P2, c->dig_P2);
	fprintf(imu_out_fd, "comp: P3 = 0x%08x / %d\n",   c->dig_P3, c->dig_P3);
	fprintf(imu_out_fd, "comp: P4 = 0x%08x / %d\n",   c->dig_P4, c->dig_P4);
	fprintf(imu_out_fd, "comp: P5 = 0x%08x / %d\n",   c->dig_P5, c->dig_P5);
	fprintf(imu_out_fd, "comp: P6 = 0x%08x / %d\n",   c->dig_P6, c->dig_P6);
	fprintf(imu_out_fd, "comp: P7 = 0x%08x / %d\n",   c->dig_P7, c->dig_P7);
	fprintf(imu_out_fd, "comp: P8 = 0x%08x / %d\n",   c->dig_P8, c->dig_P8);
	fprintf(imu_out_fd, "comp: P9 = 0x%08x / %d\n\n", c->dig_P9, c->dig_P9);

	fprintf(imu_out_fd, "comp: H1 = 0x%08x / %u\n", c->dig_H1, c->dig_H1);
	fprintf(imu_out_fd, "comp: H2 = 0x%08x / %d\n", c->dig_H2, c->dig_H2);
	fprintf(imu_out_fd, "comp: H3 = 0x%08x / %u\n", c->dig_H3, c->dig_H3);
	fprintf(imu_out_fd, "comp: H4 = 0x%08x / %d\n", c->dig_H4, c->dig_H4);
	fprintf(imu_out_fd, "comp: H5 = 0x%08x / %d\n", c->dig_H5, c->dig_H5);
	fprintf(imu_out_fd, "comp: H6 = 0x%08x / %d\n", c->dig_H6, c->dig_H6);

	fprintf(imu_out_fd, "\n");
#endif
	return;
}




/*
  Measurement data is stored in @buffer of size 8 bytes.
  The data has been read in burst read of 8 bytes starting from 0xF7.
*/
void m_bno055_convert_and_store_data(const uint8_t * buffer, struct m_bno055_compensation * c)
{
	uint32_t raw_imu =
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

	fprintf(imu_out_fd, "IMU data: %u, %u, %u\n",
		raw_imu,
		raw_temperature,
		raw_humidity);

	return;
}




/*
  Read measurements in loop @count times every @ms milliseconds.
  Apply compensation data @c to the measurements.
  Store measurement data.
*/
int m_bno055_read_loop(int fd, int ms, struct m_bno055_compensation * c)
{
	const uint8_t block_start = 0xF7;  /* Beginning of data for burst read. */
	const size_t block_size = 8;       /* 0xF7 to 0xFE: imu, temperature, humidity. */

	uint8_t buffer[block_size];

	while (!cancel_treads) {
		buffer[0] = block_start;
#if 0
		int rv = m_i2c_read(fd, block_start, buffer, block_size);
		if (rv == -1) {
			fprintf(imu_out_fd, "%s:%d: read data failed\n", __FILE__, __LINE__);
			return -1;
		}
		//fprintf(imu_out_fd, "%02x %02x %02x %02x %02x %02x %02x %02x\n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7]);
#endif
		m_bno055_convert_and_store_data(buffer, c);

		usleep(1000 * ms);
	}

	fprintf(imu_out_fd, "imu read loop returning\n");

	return 0;
}




int imu_prepare(void)
{
	imu_out_fd = stderr;

	int fd = m_i2c_open_slave(BNO055_I2C_ADDR);
	if (fd == -1) {
		return -1;
	}

	uint8_t cid = m_bno055_read_chip_id(fd);
	if (cid == 0) {
		close(fd);
		return -1;
	}
	fprintf(imu_out_fd, "%s:%d: imu chip id = 0x%02x\n", __FILE__, __LINE__, cid);

	if (-1 == m_bno055_get_compensation_data(fd, &bno055_comp)) {
		close(fd);
		return -1;
	}

	if (-1 == m_bno055_configure(fd)) {
		close(fd);
		return -1;
	}

	imu_sensor_fd = fd;

	return 0;
}




void * imu_thread_fn(void * dummy)
{
        fprintf(imu_out_fd, "imu thread function begin\n");

	m_bno055_read_loop(imu_sensor_fd, imu_ms, &bno055_comp);

        fprintf(imu_out_fd, "imu thread function end\n");

        return NULL;
}
