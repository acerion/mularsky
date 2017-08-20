/*
****************************************************************************
* Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
*
* bme280.c
* Date: 2016/07/04
* Revision: 2.0.5(Pressure and Temperature compensation code revision is 1.1
*               and Humidity compensation code revision is 1.0)
*
* Usage: Sensor Driver file for BME280 sensor
*
****************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/




#include "bme280.h"




int32_t bme280_compensate_temperature_int32(int32_t t, struct m_bme280_compensation * c)
{
	int32_t v_x1_u32r = ((((t >> 03) - ((int32_t) c->dig_T1 << 01))) * ((int32_t) c->dig_T2)) >> 11;
	int32_t v_x2_u32r = (((((t >> 04) - ((int32_t) c->dig_T1)) * ((t >> 04) - ((int32_t) c->dig_T1))) >> 12) * ((int32_t) c->dig_T3)) >> 14;
	c->fine = v_x1_u32r + v_x2_u32r;
	return (c->fine * 5 + 128) >> 8;
}




uint32_t bme280_compensate_pressure_int32(int32_t p, struct m_bme280_compensation * c)
{
	int32_t v_x1_u32 = (((int32_t) c->fine) >> 01) - (int32_t) 64000;
	int32_t v_x2_u32 = (((v_x1_u32 >> 02) * (v_x1_u32 >> 02)) >> 11) * ((int32_t) c->dig_P6);

	v_x2_u32 = v_x2_u32 + ((v_x1_u32 * ((int32_t) c->dig_P5)) << 01);
	v_x2_u32 = (v_x2_u32 >> 02) + (((int32_t) c->dig_P4) << 16);

	v_x1_u32 = (((c->dig_P3 * (((v_x1_u32 >> 02) * (v_x1_u32 >> 02)) >> 13)) >> 03) + ((((int32_t) c->dig_P2) * v_x1_u32) >> 01)) >> 18;
	v_x1_u32 = ((((32768 + v_x1_u32)) * ((int32_t) c->dig_P1)) >> 15);

	uint32_t v_pressure_u32 = (((uint32_t) (((int32_t) 1048576) - p) - (v_x2_u32 >> 12))) * 3125;
	if (v_pressure_u32 < 0x80000000) {
		/* Avoid exception caused by division by zero. */
		if (v_x1_u32 != 0) {
			v_pressure_u32 = (v_pressure_u32 << 01) / ((uint32_t) v_x1_u32);
		} else {
			return 0;
		}
	} else {
		/* Avoid exception caused by division by zero. */
		if (v_x1_u32 != 0) {
			v_pressure_u32 = (v_pressure_u32 / (uint32_t) v_x1_u32) * 2;
		} else {
			return 0;
		}

		v_x1_u32 = (((int32_t) c->dig_P9) * ((int32_t) (((v_pressure_u32 >> 03) * (v_pressure_u32 >> 03))))) >> 12;
		v_x2_u32 = (((int32_t) (v_pressure_u32 >> 02)) * ((int32_t) c->dig_P8)) >> 13;
		v_pressure_u32 = (uint32_t) ((int32_t)v_pressure_u32 + ((v_x1_u32 + v_x2_u32 + c->dig_P7) >> 04));
	}

	return v_pressure_u32;
}




uint32_t bme280_compensate_humidity_int32(int32_t h, struct m_bme280_compensation * c)
{
	int32_t v_x1_u32 = (c->fine - ((int32_t) 76800));

	v_x1_u32 = (((((h << 14) - (((int32_t) c->dig_H4) << 20) - (((int32_t) c->dig_H5) * v_x1_u32)) + ((int32_t) 16384)) >> 15) * (((((((v_x1_u32 * ((int32_t) c->dig_H6)) >> 10) * (((v_x1_u32 * ((int32_t) c->dig_H3)) >> 11) + ((int32_t) 32768))) >> 10) + ((int32_t) 2097152)) * ((int32_t) c->dig_H2) + 8192) >> 14));
	v_x1_u32 = (v_x1_u32 - (((((v_x1_u32 >> 15) * (v_x1_u32 >> 15)) >> 07) * ((int32_t) c->dig_H1)) >> 04));
	v_x1_u32 = (v_x1_u32 < 0 ? 0 : v_x1_u32);
	v_x1_u32 = (v_x1_u32 > 419430400 ? 419430400 : v_x1_u32);
	return (uint32_t) (v_x1_u32 >> 12);
}




double bme280_compensate_temperature_double(int32_t t, struct m_bme280_compensation * c)
{
	double v_x1_u32  = (((double) t) / 16384.0 - ((double) c->dig_T1) / 1024.0) * ((double) c->dig_T2);
	double v_x2_u32  = ((((double) t) / 131072.0 - ((double) c->dig_T1) / 8192.0) * (((double) t) / 131072.0 - ((double) c->dig_T1) / 8192.0)) * ((double) c->dig_T3);
	c->fine = (int32_t)(v_x1_u32 + v_x2_u32);

	return (v_x1_u32 + v_x2_u32) / 5120.0;
}




double bme280_compensate_pressure_double(int32_t p, struct m_bme280_compensation * c)
{
	double v_x1_u32 = ((double) c->fine / 2.0) - 64000.0;
	double v_x2_u32 = v_x1_u32 * v_x1_u32 * ((double) c->dig_P6) / 32768.0;
	v_x2_u32 = v_x2_u32 + v_x1_u32 * ((double) c->dig_P5) * 2.0;
	v_x2_u32 = (v_x2_u32 / 4.0) + (((double) c->dig_P4) * 65536.0);
	v_x1_u32 = (((double) c->dig_P3) * v_x1_u32 * v_x1_u32 / 524288.0 + ((double) c->dig_P2) * v_x1_u32) / 524288.0;
	v_x1_u32 = (1.0 + v_x1_u32 / 32768.0) * ((double) c->dig_P1);

	double pressure = 1048576.0 - (double) p;
	/* Avoid exception caused by division by zero. */
	if ((v_x1_u32 > 0) || (v_x1_u32 < 0)) {
		pressure = (pressure - (v_x2_u32 / 4096.0)) * 6250.0 / v_x1_u32;
	} else {
		return 0;
	}
	v_x1_u32 = ((double) c->dig_P9) * pressure * pressure / 2147483648.0;
	v_x2_u32 = pressure * ((double) c->dig_P8) / 32768.0;
	pressure = pressure + (v_x1_u32 + v_x2_u32 + ((double) c->dig_P7)) / 16.0;

	return pressure;
}




double bme280_compensate_humidity_double(int32_t h, struct m_bme280_compensation * c)
{
	double var_h = (((double) c->fine) - 76800.0);
	if ((var_h > 0) || (var_h < 0)) {
		var_h = (h - (((double) c->dig_H4) * 64.0 + ((double) c->dig_H5) / 16384.0 * var_h)) * (((double) c->dig_H2) / 65536.0 * (1.0 + ((double) c->dig_H6) / 67108864.0 * var_h * (1.0 + ((double) c->dig_H3) / 67108864.0 * var_h)));
	} else {
		return 0;
	}
	var_h = var_h * (1.0 - ((double) c->dig_H1) * var_h / 524288.0);
	if (var_h > 100.0) {
		var_h = 100.0;
	} else if (var_h < 0.0) {
		var_h = 0.0;
	}
	return var_h;
}




uint32_t bme280_compensate_pressure_int64(int32_t p, struct m_bme280_compensation * c)
{
	int64_t v_x1_s64r = ((int64_t) c->fine) - 128000;
	int64_t v_x2_s64r = v_x1_s64r * v_x1_s64r * (int64_t) c->dig_P6;
	v_x2_s64r = v_x2_s64r + ((v_x1_s64r * (int64_t) c->dig_P5) << 17);
	v_x2_s64r = v_x2_s64r + (((int64_t) c->dig_P4) << 35);
	v_x1_s64r = ((v_x1_s64r * v_x1_s64r * (int64_t) c->dig_P3) >> 8) + ((v_x1_s64r * (int64_t) c->dig_P2) << 12);
	v_x1_s64r = (((((int64_t)1) << 47) + v_x1_s64r)) * ((int64_t) c->dig_P1) >> 33;
	int64_t pressure = 1048576 - p;
	/* Avoid exception caused by division by zero. */
	if (v_x1_s64r != 0) {
		pressure = (((pressure << 31) - v_x2_s64r) * 3125) / v_x1_s64r;
	} else {
		return 0;
	}
	v_x1_s64r = (((int64_t) c->dig_P9) * (pressure >> 13) * (pressure >> 13)) >> 25;
	v_x2_s64r = (((int64_t) c->dig_P8) * pressure) >> 19;
	pressure = (((pressure + v_x1_s64r + v_x2_s64r) >> 8) + (((int64_t) c->dig_P7) << 04));

	return (uint32_t) pressure;
}
