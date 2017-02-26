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




#define BNO055_I2C_ADDR 0x28

/* BNO055 configuration registers and their contents (chip configuration). */

#define BNO055_REG_CHIP_ID         0x00   /* Read only. Value: 0xA0. */
#define BNO055_REG_ACC_ID          0x01   /* Read only. Value: 0xFB. */
#define BNO055_REG_MAG_ID          0x02   /* Read only. Value: 0x32. */
#define BNO055_REG_GYR_ID          0x03   /* Read only. Value: 0x0F. */

#define BNO055_REG_ST_RESULT       0x36
#define BNO055_REG_SYS_STATUS      0x39
#define BNO055_REG_SYS_ERR         0x3A
#define BNO055_REG_SYS_TRIGGER     0x3F   /* Write 0x01 to trigger BIST test. */

#define BNO055_REG_CALIB_STAT      0x35
#define BNO055_REG_OPR_MODE        0x3D

#define BNO055_OPR_MODE_CONFIGMODE 0x00
#define BNO055_OPR_MODE_FUS_IMU    0x08
#define BNO055_OPR_MODE_FUS_NDOF0  0x0B   /* NDOF_FMC_OFF */
#define BNO055_OPR_MODE_FUS_NDOF1  0x0C   /* NDOF */


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
static int m_bno055_read_initial(int fd);
static int m_bno055_calibrate(int fd);
static int m_bno055_read_calibration(int fd);
static int m_bno055_read(int fd);
static int m_bno055_configure(int fd);
static int m_bno055_get_compensation_data(int fd, struct m_bno055_compensation * c);
static void m_bno055_convert_and_store_data(const uint8_t * buffer, struct m_bno055_compensation * c);
static int m_bno055_read_loop(int fd, int ms, struct m_bno055_compensation * c);




/*
  Read Chip ID, other IDs, POST results and BIST results

  Table 4-2: Register Map Page 0
*/
int m_bno055_read_initial(int fd)
{
	uint8_t buffer[2] = { 0 };
	if (-1 == m_i2c_read(fd, BNO055_REG_CHIP_ID, buffer, 1)) {
		fprintf(imu_out_fd, "failed to read imu chip id\n");
		return -1;
	}
	fprintf(imu_out_fd, "imu chip id: %02X\n", buffer[0]);


	if (-1 == m_i2c_read(fd, BNO055_REG_ACC_ID, buffer, 1)) {
		fprintf(imu_out_fd, "failed to read imu acc id\n");
		return -1;
	}
	fprintf(imu_out_fd, "imu acc id:  %02X\n", buffer[0]);


	if (-1 == m_i2c_read(fd, BNO055_REG_MAG_ID, buffer, 1)) {
		fprintf(imu_out_fd, "failed to read imu mag id\n");
		return -1;
	}
	fprintf(imu_out_fd, "imu mag id:  %02X\n", buffer[0]);



	if (-1 == m_i2c_read(fd, BNO055_REG_GYR_ID, buffer, 1)) {
		fprintf(imu_out_fd, "failed to read imu gyr id\n");
		return -1;
	}
	fprintf(imu_out_fd, "imu gyr id:  %02X\n", buffer[0]);



	if (-1 == m_i2c_read(fd, BNO055_REG_ST_RESULT, buffer, 1)) {
		fprintf(imu_out_fd, "failed to read imu POST results\n");
		return -1;
	}
	fprintf(imu_out_fd, "imu POST result: %02X\n", buffer[0]);



	if (-1 == m_i2c_read(fd, BNO055_REG_SYS_STATUS, buffer, 1)) {
		fprintf(imu_out_fd, "failed to read imu SYS status before BIST\n");
		return -1;
	}
	fprintf(imu_out_fd, "imu SYS status before BIST: %02X\n", buffer[0]);



	if (-1 == m_i2c_read(fd, BNO055_REG_SYS_ERR, buffer, 1)) {
		fprintf(imu_out_fd, "failed to read imu SYS err before BIST\n");
		return -1;
	}
	fprintf(imu_out_fd, "imu SYS err before BIST:    %02X\n", buffer[0]);


#if 0
	fprintf(imu_out_fd, "imu BIST start\n");
	buffer[0] = BNO055_REG_SYS_TRIGGER;
	buffer[1] = 0x01;
	if (write(fd, buffer, 2) != 2) {
		fprintf(imu_out_fd, "imu BIST trigger failed\n");
		return -1;
	}
	do {
		sleep(1);
		if (-1 == m_i2c_read(fd, BNO055_REG_SYS_STATUS, buffer, 1)) {
			fprintf(imu_out_fd, "failed to read imu SYS status during BIST\n");
			return -1;
		}
		fprintf(imu_out_fd, "imu SYS status during BIST: %02X\n", buffer[0]);
	} while (buffer[0] == 0x04); /* 0x04 - executing selftest (4.3.58 SYS_STATUS 0x39) */

	if (-1 == m_i2c_read(fd, BNO055_REG_ST_RESULT, buffer, 1)) {
		fprintf(imu_out_fd, "failed to read imu ST result after BIST\n");
		return -1;
	}
	fprintf(imu_out_fd, "imu ST result after BIST:  %02X\n", buffer[0]);



	if (-1 == m_i2c_read(fd, BNO055_REG_SYS_STATUS, buffer, 1)) {
		fprintf(imu_out_fd, "failed to read imu SYS status after BIST\n");
		return -1;
	}
	fprintf(imu_out_fd, "imu SYS status after BIST: %02X\n", buffer[0]);



	if (-1 == m_i2c_read(fd, BNO055_REG_SYS_ERR, buffer, 1)) {
		fprintf(imu_out_fd, "failed to read imu SYS err after BIST\n");
		return -1;
	}
	fprintf(imu_out_fd, "imu SYS err after BIST:    %02X\n", buffer[0]);
#endif

	return 0;
}




int m_bno055_calibrate(int fd)
{
#if 1
	uint8_t buf[2] = { BNO055_REG_OPR_MODE, BNO055_OPR_MODE_FUS_NDOF1 };
	if (write(fd, buf, 2) != 2) {
		fprintf(imu_out_fd, "imu: failed to set oper mode\n");
		return -1;
	}
	usleep(30); /* Operating mode switching time. */
#endif

	int i = 0;
	uint8_t buffer = 0;

	while (i != 5) {
		if (-1 == m_i2c_read(fd, BNO055_REG_CALIB_STAT, &buffer, 1)) {
			fprintf(imu_out_fd, "imu: failed to read imu calibration status\n");
			return -1;
		}
		fprintf(imu_out_fd, "imu: calibration status: sys: %d, gyr: %d, acc: %d, mag: %d\n",
			(buffer & 0xC0) >> 6, (buffer & 0x30) >> 4, (buffer & 0x0C) >> 2, buffer & 0x03);


		if (buffer == 0xff) {
			i++; /* We want to have correct state of calibration for N seconds before completing calibration step. */
		} else {
			i = 0;
		}

		sleep(1);
	}

	return 0;
}




int m_bno055_read_calibration(int fd)
{
	uint8_t buffer[6] = { 0 };


	/* 3.11.4 Reuse of Calibration Profile
	   "Host system can read the offsets and radius only after a
	   full calibration is achieved and the operation mode is
	   switched to CONFIG_MODE." */
	buffer[0] = BNO055_REG_OPR_MODE;
	buffer[1] = BNO055_OPR_MODE_CONFIGMODE;
	if (write(fd, buffer, 2) != 2) {
		fprintf(imu_out_fd, "imu: failed to set oper mode before reading calibration\n");
		return -1;
	}
	usleep(30); /* Operating mode switching time. */



	/* Accelerometer offset. */
	if (-1 == m_i2c_read(fd, 0x55, buffer, 6)) {
		fprintf(imu_out_fd, "imu: failed to read imu acc offset\n");
		return -1;
	}
	fprintf(imu_out_fd, "imu: calibration data: acc offset: x = %02X%02X, y = %02X%02X, z = %02X%02X\n",
		buffer[1], buffer[0], buffer[3], buffer[2], buffer[5], buffer[4]);


	/* Mangnetometer offset. */
	if (-1 == m_i2c_read(fd, 0x5B, buffer, 6)) {
		fprintf(imu_out_fd, "imu: failed to read imu mag offset\n");
		return -1;
	}
	fprintf(imu_out_fd, "imu: calibration data: mag offset: x = %02X%02X, y = %02X%02X, z = %02X%02X\n",
		buffer[1], buffer[0], buffer[3], buffer[2], buffer[5], buffer[4]);


	/* Gyroscope offset. */
	if (-1 == m_i2c_read(fd, 0x61, buffer, 6)) {
		fprintf(imu_out_fd, "imu: failed to read imu gyro offset\n");
		return -1;
	}
	fprintf(imu_out_fd, "imu: calibration data: gyro offset: x = %02X%02X, y = %02X%02X, z = %02X%02X\n",
		buffer[1], buffer[0], buffer[3], buffer[2], buffer[5], buffer[4]);


	/* Accelerometer radius. */
	if (-1 == m_i2c_read(fd, 0x67, buffer, 2)) {
		fprintf(imu_out_fd, "imu: failed to read imu acc radius\n");
		return -1;
	}
	fprintf(imu_out_fd, "imu: calibration data: acc radius: %02X%02X\n", buffer[1], buffer[0]);


	/* Magnetometer radius. */
	if (-1 == m_i2c_read(fd, 0x69, buffer, 2)) {
		fprintf(imu_out_fd, "imu: failed to read imu mag radius\n");
		return -1;
	}
	fprintf(imu_out_fd, "imu: calibration data: mag radius: %02X%02X\n", buffer[1], buffer[0]);



	buffer[0] = BNO055_REG_OPR_MODE;
	buffer[1] = BNO055_OPR_MODE_FUS_NDOF1;
	if (write(fd, buffer, 2) != 2) {
		fprintf(imu_out_fd, "imu: failed to set oper mode after reading calibration\n");
		return -1;
	}
	usleep(30); /* Operating mode switching time. */


	return 0;
}




int m_bno055_read(int fd)
{
	uint8_t start = 0x1A; /* Beginning of Euler angles. */
	uint8_t buffer[6] = { 0 };

	while (1) {
		if (-1 == m_i2c_read(fd, start, buffer, 6)) {
			fprintf(imu_out_fd, "imu: failed to read data\n");
			return -1;
		}

		/* Table 3-22: Gyroscope unit settings
		   "1 Dps = 16 LSB" */
		fprintf(imu_out_fd, "imu: data: heading (yaw) = %d, roll = %d, pitch = %d\n",
			((int16_t) ((buffer[1] << 8) | buffer[0])) / 16,
			((int16_t) ((buffer[3] << 8) | buffer[2])) / 16,
			((int16_t) ((buffer[5] << 8) | buffer[4])) / 16);
		sleep(1);
	}

	return 0;
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

	int fd = m_i2c_open_slave(3, BNO055_I2C_ADDR);
	if (fd == -1) {
		return -1;
	}

	if (-1 == m_bno055_read_initial(fd)) {
		close(fd);
		return -1;
	}

	if (-1 == m_bno055_calibrate(fd)) {
		close(fd);
		return -1;
	}


	if (-1 == m_bno055_read_calibration(fd)) {
		close(fd);
		return -1;
	}


	if (-1 == m_bno055_read(fd)) {
		close(fd);
		return -1;
	}



	return -1;

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
