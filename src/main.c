#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "m_bme280.h"
#include "m_i2c.h"




#define BME280_I2C_ADDR 0x77




static int pressure_fd = 0;
static int32_t pressure_read_count = 0;
static struct m_bme280_compensation bme280_comp;




void m_atexit(void)
{
	if (pressure_fd) {
		close(pressure_fd);
		pressure_fd = 0;
	}
	return;
}



int main(int argc, char ** argv)
{
	if (argc != 2) {
		fprintf(stderr, "%s <numer of reads>\n", argv[0]);
		return -1;
	}

	atexit(m_atexit);

	pressure_read_count = atoi(argv[1]);

	pressure_fd = m_i2c_open_slave(BME280_I2C_ADDR);
	if (pressure_fd == -1) {
		exit(EXIT_FAILURE);
	}

	uint8_t cid = m_bme280_read_chip_id(pressure_fd);
	if (cid == 0) {
		exit(EXIT_FAILURE);
	}
	fprintf(stderr, "%s:%d: pressure chip id = 0x%02x\n", __FILE__, __LINE__, cid);

	if (-1 == m_bme280_get_compensation_data(pressure_fd, &bme280_comp)) {
		exit(EXIT_FAILURE);
	}

	if (-1 == m_bme280_configure(pressure_fd)) {
		exit(EXIT_FAILURE);
	}

	if (-1 == m_bme280_read_loop(pressure_fd, pressure_read_count, 1000, &bme280_comp)) {
		exit(EXIT_FAILURE);
	}

	close(pressure_fd);
	return 0;
}
