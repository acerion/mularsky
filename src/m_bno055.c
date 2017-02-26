#define _BSD_SOURCE /* usleep() */

#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>

#include "m_bno055.h"
#include "m_i2c.h"




int imu_sensor_fd = 0;

extern bool cancel_treads;


static FILE * imu_out_fd;
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




static int m_bno055_read_initial(int fd);
static int m_bno055_calibrate(int fd);
static int m_bno055_read_calibration(int fd);
static int m_bno055_run_bist(int fd);
static int m_bno055_configure(int fd);
static void m_bno055_convert_and_store_data(const uint8_t * buffer);
static int m_bno055_read_loop(int fd, int ms);




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
	fprintf(imu_out_fd, "imu SYS status: %02X\n", buffer[0]);



	if (-1 == m_i2c_read(fd, BNO055_REG_SYS_ERR, buffer, 1)) {
		fprintf(imu_out_fd, "failed to read imu SYS err before BIST\n");
		return -1;
	}
	fprintf(imu_out_fd, "imu SYS err:    %02X\n", buffer[0]);



	return 0;
}




int m_bno055_run_bist(int fd)
{
	fprintf(imu_out_fd, "imu BIST start\n");

	uint8_t buffer[2] = { 0 };
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




/*
  Measurement data is stored in @buffer of size 45 bytes.
  The data has been read in burst read of 45 bytes starting from 0x08.
*/
void m_bno055_convert_and_store_data(const uint8_t * buffer)
{
	/* Table 3-22: Gyroscope unit settings
	   "1 Dps = 16 LSB" */
	fprintf(imu_out_fd, "imu: data:" \

		"yaw=%d, roll=%d, pitch=%d " \

		"temp=%d " \

		"\n",

		((int16_t) ((buffer[19] << 8) | buffer[18])) / 16,
		((int16_t) ((buffer[21] << 8) | buffer[20])) / 16,
		((int16_t) ((buffer[23] << 8) | buffer[22])) / 16,

		(int8_t) buffer[44]

		);

	return;
}




/*
  Read measurements in loop every @ms milliseconds.
  Store measurement data.
*/
int m_bno055_read_loop(int fd, int ms)
{
	uint8_t buffer[45] = { 0 };
	const uint8_t start = 0x08; /* Beginning of Data area. */

	while (!cancel_treads) {
		buffer[0] = start;

		if (-1 == m_i2c_read(fd, start, buffer, sizeof (buffer))) {
			fprintf(imu_out_fd, "imu: failed to read data\n");
			return -1;
		}

		m_bno055_convert_and_store_data(buffer);

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


	if (-1 == m_bno055_configure(fd)) {
		close(fd);
		return -1;
	}


#if 0
	if (-1 == m_bno055_run_bist(fd)) {
		close(fd);
		return -1;
	}
#endif


	imu_sensor_fd = fd;

	return 0;
}




void * imu_thread_fn(void * dummy)
{
        fprintf(imu_out_fd, "imu thread function begin\n");

	m_bno055_read_loop(imu_sensor_fd, imu_ms);

        fprintf(imu_out_fd, "imu thread function end\n");

        return NULL;
}
