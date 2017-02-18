#ifndef H_M_BME280
#define H_M_BME280




#include <stdint.h>




struct m_bme280_compensation {
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;

	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;

	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;

	int32_t fine;
};




uint8_t m_bme280_read_chip_id(int fd);
int m_bme280_configure(int fd);
int m_bme280_get_compensation_data(int fd, struct m_bme280_compensation * c);
void m_bme280_convert_and_store_data(const uint8_t * buffer, struct m_bme280_compensation * c);
int m_bme280_read_loop(int fd, int32_t count, int ms, struct m_bme280_compensation * c);




#endif /* #ifndef H_M_BME280 */
