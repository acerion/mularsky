/** \mainpage
*
****************************************************************************
* Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
*
* File : bme280.h
*
* Date : 2016/07/04
*
* Revision : 2.0.5(Pressure and Temperature compensation code revision is 1.1
*               and Humidity compensation code revision is 1.0)
*
* Usage: Sensor Driver for BME280 sensor
*
****************************************************************************
*
* \section License
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
/*! \file bme280.h
    \brief BME280 Sensor Driver Support Header File */
#ifndef __BME280_H__
#define __BME280_H__




#include <stdint.h>
#include <limits.h>
#include "m_bme280.h"




#if defined(LONG_MAX) && LONG_MAX == 0x7fffffffffffffffL
#define BME280_64BITSUPPORT_PRESENT
#elif defined(LLONG_MAX) && (LLONG_MAX == 0x7fffffffffffffffLL)
#define BME280_64BITSUPPORT_PRESENT
#else
#warning Either the correct data type for signed 64 bit integer could not be found, or 64 bit integers are not supported in your environment.
#warning The API will only offer 32 bit pressure calculation. This will slightly impede accuracy(noise of ~1 pascal RMS will be added to output).
#warning If 64 bit integers are supported on your platform, please set s64 manually and "#define(BME280_64BITSUPPORT_PRESENT)" manually.
#endif




/**
 * @brief Convert uncompensated temperature to compensated value
 * @note Returns the value in 0.01 degree Centigrade
 * Output value of "5123" equals 51.23 DegC.
 *
 * @param t: value of uncompensated temperature
 * @param c: compensation data
 *
 * @return Returns the actual temperature value
 */
int32_t bme280_compensate_temperature_int32(int32_t t, struct m_bme280_compensation * c);




/**
 * @brief Convert uncompensated pressure to compensated value
 * @note Returns the value in Pascal(Pa)
 * Output value of "96386" equals 96386 Pa =
 * 963.86 hPa = 963.86 millibar
 *
 * @param p: value of uncompensated pressure
 * @param c: compensation data
 *
 * @return Returns the actual pressure value
 */
uint32_t bme280_compensate_pressure_int32(int32_t p, struct m_bme280_compensation * c);




/**
 * @brief Convert uncompensated humidity to compensated value
 * @note Returns the value in %rH as unsigned 32bit integer
 * in Q22.10 format (22 integer 10 fractional bits).
 * @note An output value of 42313
 * represents 42313 / 1024 = 41.321 %rH
 *
 * @param h: value of uncompensated humidity
 * @param c: compensation data
 *
 * @return Return the actual relative humidity value
 */
uint32_t bme280_compensate_humidity_int32(int32_t h, struct m_bme280_compensation * c);




/**
 * @brief Convert uncompensated temperature to compensated value
 * @note returns the value in Degree centigrade
 * @note Output value of "51.23" equals 51.23 DegC.
 *
 * @param t: value of uncompensated temperature
 * @param c: compensation data
 *
 * @return  Return the actual temperature value
 */
double bme280_compensate_temperature_double(int32_t t, struct m_bme280_compensation * c);




/**
 * @brief Convert uncompensated pressure to compensated value
 * @note Returns pressure in Pa as double.
 * @note Output value of "96386.2"
 * equals 96386.2 Pa = 963.862 hPa.
 *
 * @param p: value of uncompensated pressure
 * @param c: compensated data
 *
 * @return Return the actual pressure value
 */
double bme280_compensate_pressure_double(int32_t p, struct m_bme280_compensation * c);




/**
 * @brief Convert uncompensated humidity to compensated value
 * @note Returns the value in relative humidity (%rH)
 * @note Output value of "42.12" equals 42.12 %rH
 *
 * @param h: value of uncompensated humidity
 * @param c: compensation data
 *
 * @return Return the actual humidity value
 */
double bme280_compensate_humidity_double(int32_t h, struct m_bme280_compensation * c);




/**
 * @brief Convert compensated pressure to compensated value
 * @note Returns the value in Pa as unsigned 32 bit
 * integer in Q24.8 format (24 integer bits and
 * 8 fractional bits).
 * @note Output value of "24674867"
 * represents 24674867 / 256 = 96386.2 Pa = 963.862 hPa
 *
 * @param p: value of uncompensated temperature
 * @param c: compensation data
 *
 * @return Return the actual pressure value
 */
uint32_t bme280_compensate_pressure_int64(int32_t p, struct m_bme280_compensation * c);




#endif /* #ifndef __BME280_H__ */
