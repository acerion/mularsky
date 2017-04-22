#define _BSD_SOURCE /* usleep() */

#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <time.h>

#include "m_bno055.h"
#include "m_i2c.h"
#include "m_misc.h"



int imu_sensor_fd = 0;

extern bool cancel_treads;
extern time_t global_time;
extern int imu_led_time_ms;


static FILE * imu_out_fd;
static const int imu_ms = 10; /* [milliseconds] */
static const char * data_filename = "imu.txt";



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

#define BNO055_REG_AXIS_MAP_CONFIG 0x41   /* Axis remapping. */
#define BNO055_REG_AXIS_MAP_SIGN   0x42   /* Axis sign. */


#define BNO055_OPR_MODE_CONFIGMODE 0x00
#define BNO055_OPR_MODE_FUS_IMU    0x08
#define BNO055_OPR_MODE_FUS_NDOF0  0x0B   /* NDOF_FMC_OFF */
#define BNO055_OPR_MODE_FUS_NDOF1  0x0C   /* NDOF */
#define BNO055_OPR_MODE_WORK_MODE  BNO055_OPR_MODE_FUS_NDOF1


#define BNO055_MANUAL_CALIBRATION 0
#define M_BNO055_RUN_BIST 0

static int m_bno055_read_initial(int fd);
#if M_BNO055_MANUAL_CALIBRATION
static int m_bno055_calibrate_manually(int fd);
#else
static int m_bno055_calibrate_from_data(int fd);
#endif
static int m_bno055_get_overall_status(int fd);
static int m_bno055_read_calibration(int fd);
#if M_BNO055_RUN_BIST
static int m_bno055_run_bist(int fd);
#endif
static int m_bno055_configure(int fd);
static void m_bno055_convert_and_store_data(const uint8_t * buffer);
static int m_bno055_read_loop(int fd, int ms);




int m_bno055_reset(int fd)
{
	uint8_t buffer[2] = { BNO055_REG_SYS_TRIGGER, 0x20 };
	if (write(fd, buffer, 2) != 2) {
		fprintf(imu_out_fd, "imu: reset failed\n");
		return -1;
	}

	sleep(1);
	errno = 0;
	fprintf(imu_out_fd, "imu: reset performed\n");

	return 0;
}




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


	return m_bno055_get_overall_status(fd);
}




int m_bno055_get_overall_status(int fd)
{
	uint8_t buffer = { 0 };


	if (-1 == m_i2c_read(fd, BNO055_REG_ST_RESULT, &buffer, 1)) {
		fprintf(imu_out_fd, "failed to read imu POST results\n");
		return -1;
	}
	fprintf(imu_out_fd, "imu POST result: %02X\n", buffer);


	if (-1 == m_i2c_read(fd, BNO055_REG_SYS_STATUS, &buffer, 1)) {
		fprintf(imu_out_fd, "failed to read imu SYS status before BIST\n");
		return -1;
	}
	fprintf(imu_out_fd, "imu SYS status:  %02X\n", buffer);


	if (-1 == m_i2c_read(fd, BNO055_REG_SYS_ERR, &buffer, 1)) {
		fprintf(imu_out_fd, "failed to read imu SYS err before BIST\n");
		return -1;
	}
	fprintf(imu_out_fd, "imu SYS err:     %02X\n", buffer);


	return 0;
}



#if M_BNO055_RUN_BIST

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

#endif /* #if M_BNO055_RUN_BIST */



#if M_BNO055_MANUAL_CALIBRATION

int m_bno055_calibrate_manually(int fd)
{
#if 1
	usleep(30);
	uint8_t buf[2] = { BNO055_REG_OPR_MODE, BNO055_OPR_MODE_WORK_MODE };
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


		if ((buffer & (0x30 | 0x0c)) == 0x3c) {
			i++; /* We want to have correct state of calibration for N seconds before completing calibration step. */
		} else {
			i = 0;
		}

		sleep(1);
	}

	return 0;
}


#else


/*
  3.11.4 Reuse of Calibration Profile
*/
int m_bno055_calibrate_from_data(int fd)
{
	fprintf(imu_out_fd, "imu: calibrating from data\n");

	usleep(30);
	uint8_t buf[2] = { BNO055_REG_OPR_MODE, BNO055_OPR_MODE_CONFIGMODE };
	if (write(fd, buf, 2) != 2) {
		fprintf(imu_out_fd, "imu: failed to set oper mode\n");
		return -1;
	}
	usleep(30); /* Operating mode switching time. */


	uint8_t buffer[] = { 0x55,        0xF1, 0xFF, 0x0A, 0x00, 0x08, 0x00, 0xE4, 0xFD, 0xC6, 0xFF, 0x77, 0xFF, 0xFF, 0xFF, 0xFC, 0xFF, 0xFF, 0xFF, 0xE8, 0x03, 0x73, 0x02, };
	if (write(fd, buffer, sizeof (buffer)) != sizeof (buffer)) {
		fprintf(imu_out_fd, "imu: failed to write calibration data\n");
		return -1;
	}

	usleep(30);
	buf[0] = BNO055_REG_OPR_MODE;
	buf[1] = BNO055_OPR_MODE_WORK_MODE;
	if (write(fd, buf, 2) != 2) {
		fprintf(imu_out_fd, "imu: failed to set oper mode\n");
		return -1;
	}
	usleep(30); /* Operating mode switching time. */

	return 0;
}

#endif /* #if M_BNO055_MANUAL_CALIBRATION */



int m_bno055_read_calibration(int fd)
{
	uint8_t buffer[22] = { 0 };

#if 1
	/* 3.11.4 Reuse of Calibration Profile
	   "Host system can read the offsets and radius only after a
	   full calibration is achieved and the operation mode is
	   switched to CONFIG_MODE." */
	usleep(30);
	buffer[0] = BNO055_REG_OPR_MODE;
	buffer[1] = BNO055_OPR_MODE_CONFIGMODE;
	if (write(fd, buffer, 2) != 2) {
		fprintf(imu_out_fd, "imu: failed to set oper mode before reading calibration\n");
		return -1;
	}
	usleep(30); /* Operating mode switching time. */
#endif

	if (-1 == m_i2c_read(fd, 0x55, buffer, sizeof (buffer))) {
		fprintf(imu_out_fd, "imu: failed to read imu calibration data\n");
		return -1;
	}

	fprintf(imu_out_fd, "imu: calibration data:\n");
	fprintf(imu_out_fd, "acc off x,acc off y,acc off z,mag off x,mag off y,mag off z,gyro off x,gyro off y,gyro off z,acc radius,mag radius\n");

	/* Accelerometer offset. */
	fprintf(imu_out_fd, "%02X%02X,%02X%02X,%02X%02X,", buffer[1], buffer[0], buffer[3], buffer[2], buffer[5], buffer[4]);

	/* Magnetometer offset. */
	fprintf(imu_out_fd, "%02X%02X,%02X%02X,%02X%02X,", buffer[7], buffer[6], buffer[9], buffer[8], buffer[11], buffer[10]);

	/* Gyroscope offset. */
	fprintf(imu_out_fd, "%02X%02X,%02X%02X,%02X%02X,", buffer[13], buffer[12], buffer[15], buffer[14], buffer[17], buffer[16]);

	/* Accelerometer radius. */
	fprintf(imu_out_fd, "%02X%02X,", buffer[19], buffer[18]);

	/* Magnetometer radius. */
	fprintf(imu_out_fd, "%02X%02X", buffer[21], buffer[20]);

	fprintf(imu_out_fd, "\n");


	/* This is for copy-paste of calibration data into C code.
	   0x55 is address of first register with calibration data. */
	fprintf(imu_out_fd, "uint8_t buffer[] = { 0x55,        ");
	for (int i = 0; i < sizeof (buffer); i++) {
		fprintf(imu_out_fd, "0x%02X, ", buffer[i]);
	}
	fprintf(imu_out_fd, "};\n");


	usleep(30);
	buffer[0] = BNO055_REG_OPR_MODE;
	buffer[1] = BNO055_OPR_MODE_WORK_MODE;
	if (write(fd, buffer, 2) != 2) {
		fprintf(imu_out_fd, "imu: failed to set oper mode after reading calibration\n");
		return -1;
	}
	usleep(30); /* Operating mode switching time. */


	return 0;
}




int m_bno055_configure(int fd)
{
	uint8_t buffer[2];

	fprintf(imu_out_fd, "imu: configuring\n");

	usleep(30);
	buffer[0] = BNO055_REG_OPR_MODE;
	buffer[1] = BNO055_OPR_MODE_CONFIGMODE ;
	if (write(fd, buffer, 2) != 2) {
		fprintf(imu_out_fd, "imu: failed to set oper mode\n");
		return -1;
	}
	usleep(30); /* Operating mode switching time. */


#if 1
	/* Axis remapping. */
	buffer[0] = BNO055_REG_AXIS_MAP_CONFIG;
	buffer[1] = (0x01 << 4) | (0x02 << 2) | (0x00 << 0);
	///buffer[1] = (0x00 << 4) | (0x02 << 2) | (0x01 << 0);
	//buffer[1] = (0x01 << 4) | (0x00 << 2) | (0x02 << 0);
	//buffer[1] = (0x00 << 4) | (0x01 << 2) | (0x02 << 0);

	if (write(fd, buffer, 2) != 2) {
		fprintf(imu_out_fd, "imu: failed to remap axis\n");
		return -1;
	}
#endif


	usleep(30);
	buffer[0] = BNO055_REG_OPR_MODE;
	buffer[1] = BNO055_OPR_MODE_WORK_MODE;
	if (write(fd, buffer, 2) != 2) {
		fprintf(imu_out_fd, "imu: failed to set oper mode after configuration\n");
		return -1;
	}
	usleep(30); /* Operating mode switching time. */



	return 0;
}




/*
  Measurement data is stored in @buffer of size 46 bytes.
  The data has been read in burst read of 46 bytes starting from 0x08.
*/
void m_bno055_convert_and_store_data(const uint8_t * buffer)
{
	fprintf(imu_out_fd, "imu@%lu:" \

		"acc=%d,%d,%d " \
		"mag=%d,%d,%d " \
		"gyr=%d,%d,%d " \
		"eul=%d,%d,%d " \
		"qua=%d,%d,%d,%d " \
		"lia=%d,%d,%d " \
		"grv=%d,%d,%d " \
		"temp=%d " \
		"calib=0x%x"  \

		"\n",

		global_time,

		/* ACC_DATA */
		/* Table 3-17: Accelerometer Unit settings
		   1 m/s 2 = 100 LSB. */
		((int16_t) ((buffer[1] << 8) | buffer[0])),
		((int16_t) ((buffer[3] << 8) | buffer[2])),
		((int16_t) ((buffer[5] << 8) | buffer[4])),

		/* MAG_DATA */
		/* Table 3-19: Magnetometer Unit settings
		   1 uT = 16 LSB */
		((int16_t) ((buffer[7] << 8) | buffer[6])),
		((int16_t) ((buffer[9] << 8) | buffer[8])),
		((int16_t) ((buffer[11] << 8) | buffer[10])),

		/* GYR_DATA */
		/* Table 3-22: Gyroscope unit settings
		   1 Dps = 16 LSB */
		((int16_t) ((buffer[13] << 8) | buffer[12])),
		((int16_t) ((buffer[15] << 8) | buffer[14])),
		((int16_t) ((buffer[17] << 8) | buffer[16])),

		/* EUL */
		/* Table 3-29: Euler angle data representation
		   1 degree = 16 LSB */
		((int16_t) ((buffer[19] << 8) | buffer[18])) / 16,
		((int16_t) ((buffer[21] << 8) | buffer[20])) / 16,
		((int16_t) ((buffer[23] << 8) | buffer[22])) / 16,

		/* QUA_Data */
		/* Table 3-31: Quaternion data representation
		   1 Quaternion (unit less) = 2^14 LSB */
		((int16_t) ((buffer[25] << 8) | buffer[24])),
		((int16_t) ((buffer[27] << 8) | buffer[26])),
		((int16_t) ((buffer[29] << 8) | buffer[28])),
		((int16_t) ((buffer[31] << 8) | buffer[30])),

		/* LIA_Data */
		/* Table 3-33: Linear Acceleration data representation
		   1 m/s 2 = 100 LSB */
		((int16_t) ((buffer[33] << 8) | buffer[32])),
		((int16_t) ((buffer[35] << 8) | buffer[34])),
		((int16_t) ((buffer[37] << 8) | buffer[36])),

		/* GRV_Data */
		/* Table 3-35: Gravity Vector data representation
		   1 m/s 2 = 100 LSB */
		((int16_t) ((buffer[39] << 8) | buffer[38])),
		((int16_t) ((buffer[41] << 8) | buffer[40])),
		((int16_t) ((buffer[43] << 8) | buffer[42])),

		/* TEMP */
		/* Table 3-37: Temperature data representation
		   1°C = 1 LSB */
		(int8_t) buffer[44],

		buffer[45]

		);

	return;
}




/*
  Read measurements in loop every @ms milliseconds.
  Store measurement data.
*/
int m_bno055_read_loop(int fd, int ms)
{
	uint8_t buffer[46] = { 0 };
	const uint8_t start = 0x08; /* Beginning of Data area. */

	while (!cancel_treads) {
		buffer[0] = start;
		if (-1 == m_i2c_read(fd, start, buffer, sizeof (buffer))) {
			fprintf(imu_out_fd, "imu: failed to read data\n");
			return -1;
		}

		m_bno055_convert_and_store_data(buffer);

		usleep(USECS_PER_MSEC * ms);
	}

	fprintf(imu_out_fd, "imu read loop returning\n");

	return 0;
}




int imu_prepare(char const * dirpath)
{
	if (dirpath == NULL) {
		imu_out_fd = stderr;
	} else {
		char buffer[64] = { 0 };
		snprintf(buffer, sizeof (buffer), "%s/%s", dirpath, data_filename);
		imu_out_fd = fopen(buffer, "w");
		//setvbuf(imu_out_fd, NULL, _IONBF, 0);
	}

	int fd = m_i2c_open_slave(3, BNO055_I2C_ADDR);
	if (fd == -1) {
		return -1;
	}


	if (-1 == m_bno055_reset(fd)) {
		close(fd);
		return -1;
	}


	if (-1 == m_bno055_read_initial(fd)) {
		close(fd);
		return -1;
	}

#if M_BNO055_MANUAL_CALIBRATION
	if (-1 == m_bno055_calibrate_manually(fd)) {
		close(fd);
		return -1;
	}
#else
	if (-1 == m_bno055_calibrate_from_data(fd)) {
		close(fd);
		return -1;
	}
#endif

	if (-1 == m_bno055_read_calibration(fd)) {
		close(fd);
		return -1;
	}

	if (-1 == m_bno055_configure(fd)) {
		close(fd);
		return -1;
	}


#if M_BNO055_RUN_BIST
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

	if (-1 == m_bno055_get_overall_status(imu_sensor_fd)) {
		fprintf(imu_out_fd, "imu: thread function: failed to get overall imu status\n");
		close(imu_sensor_fd);
		return NULL;
	}


	imu_led_time_ms = BLINK_OK;

	m_bno055_read_loop(imu_sensor_fd, imu_ms);

	fprintf(imu_out_fd, "imu thread function end\n");

	if (imu_out_fd && imu_out_fd != stderr) {
		fclose(imu_out_fd);
		imu_out_fd = NULL;
	}

	return NULL;
}
