/*
 * BME280.cpp
 *
 * Copyright (C) 2023 Wanhive Systems Private Limited (info@wanhive.com)
 *
 * SPDX License Identifier: GPL-3.0-or-later
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/*
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "BME280.h"
#include <wanhive/base/common/Exception.h>
#include <wanhive/base/Timer.h>
#include <cstdint>

/*! Register Address */
#define BME280_REG_CHIP_ID                        UINT8_C(0xD0)
#define BME280_REG_RESET                          UINT8_C(0xE0)
#define BME280_REG_TEMP_PRESS_CALIB_DATA          UINT8_C(0x88)
#define BME280_REG_HUMIDITY_CALIB_DATA            UINT8_C(0xE1)
#define BME280_REG_CTRL_HUM                       UINT8_C(0xF2)
#define BME280_REG_STATUS                         UINT8_C(0xF3)
#define BME280_REG_PWR_CTRL                       UINT8_C(0xF4)
#define BME280_REG_CTRL_MEAS                      UINT8_C(0xF4)
#define BME280_REG_CONFIG                         UINT8_C(0xF5)
#define BME280_REG_DATA                           UINT8_C(0xF7)

/*! Macros related to size */
#define BME280_LEN_TEMP_PRESS_CALIB_DATA          UINT8_C(26)
#define BME280_LEN_HUMIDITY_CALIB_DATA            UINT8_C(7)
#define BME280_LEN_P_T_H_DATA                     UINT8_C(8)

#define BME280_SENSOR_MODE_MSK                    UINT8_C(0x03)
#define BME280_SENSOR_MODE_POS                    UINT8_C(0x00)

/*! Soft reset command */
#define BME280_SOFT_RESET_COMMAND                 UINT8_C(0xB6)

#define BME280_CTRL_HUM_MSK                       UINT8_C(0x07)
#define BME280_CTRL_HUM_POS                       UINT8_C(0x00)
#define BME280_CTRL_PRESS_MSK                     UINT8_C(0x1C)
#define BME280_CTRL_PRESS_POS                     UINT8_C(0x02)
#define BME280_CTRL_TEMP_MSK                      UINT8_C(0xE0)
#define BME280_CTRL_TEMP_POS                      UINT8_C(0x05)

/*! Measurement delay calculation macros  */
#define BME280_MEAS_OFFSET                        UINT16_C(1250)
#define BME280_MEAS_DUR                           UINT16_C(2300)
#define BME280_PRES_HUM_MEAS_OFFSET               UINT16_C(575)
#define BME280_MEAS_SCALING_FACTOR                UINT16_C(1000)
#define BME280_STARTUP_DELAY                      UINT16_C(2000)

/*! Length macros */
#define BME280_MAX_LEN                            UINT8_C(10)

#define BME280_STANDBY_MSK                        UINT8_C(0xE0)
#define BME280_STANDBY_POS                        UINT8_C(0x05)

/*! Bit shift macros */
#define BME280_12_BIT_SHIFT                       UINT8_C(12)
#define BME280_8_BIT_SHIFT                        UINT8_C(8)
#define BME280_4_BIT_SHIFT                        UINT8_C(4)

#define BME280_FILTER_MSK                         UINT8_C(0x1C)
#define BME280_FILTER_POS                         UINT8_C(0x02)

/*! Macro to combine two 8 bit data's to form a 16 bit data */
#define BME280_CONCAT_BYTES(msb, lsb)             (((uint16_t)msb << 8) | (uint16_t)lsb)

/*! Macro to SET and GET BITS of a register */
#define BME280_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     ((data << bitname##_POS) & bitname##_MSK))

#define BME280_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     (data & bitname##_MSK))

#define BME280_GET_BITS(reg_data, bitname)        ((reg_data & (bitname##_MSK)) >> \
                                                   (bitname##_POS))
#define BME280_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))

/* To identify osr settings selected by user */
#define OVERSAMPLING_SETTINGS    UINT8_C(0x07)

/* To identify filter and standby settings selected by user */
#define FILTER_STANDBY_SETTINGS  UINT8_C(0x18)

namespace wanhive {

BME280::BME280(unsigned int bus, unsigned int address) :
		SMBus(bus, address) {
	setup();
}

BME280::BME280(const char *path, unsigned int address) :
		SMBus(path, address) {
	setup();
}

BME280::~BME280() {

}

void BME280::setup() {
	/* Read the chip-id of bme280 sensor */
	auto chip_id = SMBus::readByte(BME280_REG_CHIP_ID);
	/* Check for chip id validity */
	if (chip_id == CHIP_ID) {
		dev.chipId = chip_id;
		/* Reset the sensor */
		reset();
		/* Read the calibration data */
		calibrate();
	} else {
		throw Exception(EX_RESOURCE);
	}
}

void BME280::reset() const {
	uint8_t reg_addr = BME280_REG_RESET;
	uint8_t status_reg = 0;
	uint8_t try_run = 5;

	/* 0xB6 is the soft reset command */
	uint8_t soft_rst_cmd = BME280_SOFT_RESET_COMMAND;

	/* Write the soft reset command in the sensor */
	SMBus::write(reg_addr, soft_rst_cmd);

	/* If NVM not copied yet, Wait for NVM to copy */
	do {
		/* As per data sheet - Table 1, startup time is 2 ms. */
		Timer::sleep(BME280_STARTUP_DELAY / 1000);
		status_reg = SMBus::readByte(BME280_REG_STATUS);

	} while ((try_run--) && (status_reg & STATUS_IM_UPDATE));

	if (status_reg & STATUS_IM_UPDATE) {
		throw Exception(EX_OPERATION);
	}
}

void BME280::getConfiguration(BME280Config &conf) {
	uint8_t reg_data[4];
	SMBus::read(BME280_REG_CTRL_HUM, 4, reg_data);
	parseConfiguration(reg_data, conf);
}

void BME280::setConfiguration(const BME280Config &conf) {
	setConfiguration(conf, SEL_ALL_SETTINGS);
}

void BME280::setConfiguration(const BME280Config &conf, unsigned char what) {
	uint8_t sensor_mode;
	sensor_mode = getPowerMode();

	if (sensor_mode != POWERMODE_SLEEP) {
		sleep();
	}

	/* Check if user wants to change oversampling
	 * settings
	 */
	if (OVERSAMPLING_SETTINGS & what) {
		setOversampling(what, conf);
	}

	/* Check if user wants to change filter and/or
	 * standby settings
	 */
	if (FILTER_STANDBY_SETTINGS & what) {
		setFilterAndStandby(what, conf);
	}
}

unsigned char BME280::getPowerMode() {
	/* Read the power mode register */
	auto mode = SMBus::readByte(BME280_REG_PWR_CTRL);
	/* Return the power mode */
	return BME280_GET_BITS_POS_0(mode, BME280_SENSOR_MODE);
}

void BME280::setPowerMode(unsigned char mode) {
	uint8_t last_set_mode = getPowerMode();

	/* If the sensor is not in sleep mode put the device to sleep
	 * mode
	 */
	if (last_set_mode != POWERMODE_SLEEP) {
		sleep();
	}

	writePowerMode(mode);
}

unsigned char BME280::getStatus() const {
	return SMBus::readByte(BME280_REG_STATUS);
}

void BME280::getData(BME280Data &data) {
	getData(data, SENSE_ALL);
}

void BME280::getData(BME280Data &data, unsigned char what) {
	/* Array to store the pressure, temperature and humidity data read from
	 * the sensor
	 */
	uint8_t reg_data[BME280_LEN_P_T_H_DATA] = { 0 };
	raw = { 0, 0, 0 };

	/* Read the pressure and temperature data from the sensor */
	SMBus::read(BME280_REG_DATA, BME280_LEN_P_T_H_DATA, reg_data);
	/* Parse the read data from the sensor */
	parseRawData(reg_data);

	/* Compensate the pressure and/or temperature and/or
	 * humidity data from the sensor
	 */
	compensate(what, data);
}

unsigned BME280::calculateDelay(const BME280Config &conf) const noexcept {
	uint8_t temp_osr;
	uint8_t pres_osr;
	uint8_t hum_osr;

	/* Array to map OSR config register value to actual OSR */
	uint8_t osr_sett_to_act_osr[] = { 0, 1, 2, 4, 8, 16 };

	/* Mapping osr settings to the actual osr values e.g. 0b101 -> osr X16 */
	if (conf.os.temperature <= OVERSAMPLING_16X) {
		temp_osr = osr_sett_to_act_osr[conf.os.temperature];
	} else {
		temp_osr = OVERSAMPLING_MAX;
	}

	if (conf.os.pressure <= OVERSAMPLING_16X) {
		pres_osr = osr_sett_to_act_osr[conf.os.pressure];
	} else {
		pres_osr = OVERSAMPLING_MAX;
	}

	if (conf.os.humidity <= OVERSAMPLING_16X) {
		hum_osr = osr_sett_to_act_osr[conf.os.humidity];
	} else {
		hum_osr = OVERSAMPLING_MAX;
	}

	return (uint32_t) ((BME280_MEAS_OFFSET + (BME280_MEAS_DUR * temp_osr)
			+ ((BME280_MEAS_DUR * pres_osr) + BME280_PRES_HUM_MEAS_OFFSET)
			+ ((BME280_MEAS_DUR * hum_osr) + BME280_PRES_HUM_MEAS_OFFSET)));
}

void BME280::compensate(unsigned char what, BME280Data &result) {
	/* Initialize to zero */
	result.temperature = 0;
	result.pressure = 0;
	result.humidity = 0;

	/* If pressure or temperature component is selected */
	if (what & (SENSE_PRESSURE | SENSE_TEMPERATURE | SENSE_HUMIDITY)) {
		/* Compensate the temperature data */
		result.temperature = compensateTemperature();
	}

	if (what & SENSE_PRESSURE) {
		/* Compensate the pressure data */
		result.pressure = compensatePressure();
	}

	if (what & SENSE_HUMIDITY) {
		/* Compensate the humidity data */
		result.humidity = compensateHumidity();
	}
}

void BME280::calibrate() {
	uint8_t reg_addr = BME280_REG_TEMP_PRESS_CALIB_DATA;
	/* Array to store calibration data */
	uint8_t calib_data[BME280_LEN_TEMP_PRESS_CALIB_DATA] = { 0 };

	/* Read the calibration data from the sensor */
	SMBus::read(reg_addr, BME280_LEN_TEMP_PRESS_CALIB_DATA, calib_data);

	/* Parse temperature and pressure calibration data and store
	 * it in device structure
	 */
	calib.dig_t1 = BME280_CONCAT_BYTES(calib_data[1], calib_data[0]);
	calib.dig_t2 = (int16_t) BME280_CONCAT_BYTES(calib_data[3], calib_data[2]);
	calib.dig_t3 = (int16_t) BME280_CONCAT_BYTES(calib_data[5], calib_data[4]);
	calib.dig_p1 = BME280_CONCAT_BYTES(calib_data[7], calib_data[6]);
	calib.dig_p2 = (int16_t) BME280_CONCAT_BYTES(calib_data[9], calib_data[8]);
	calib.dig_p3 = (int16_t) BME280_CONCAT_BYTES(calib_data[11],
			calib_data[10]);
	calib.dig_p4 = (int16_t) BME280_CONCAT_BYTES(calib_data[13],
			calib_data[12]);
	calib.dig_p5 = (int16_t) BME280_CONCAT_BYTES(calib_data[15],
			calib_data[14]);
	calib.dig_p6 = (int16_t) BME280_CONCAT_BYTES(calib_data[17],
			calib_data[16]);
	calib.dig_p7 = (int16_t) BME280_CONCAT_BYTES(calib_data[19],
			calib_data[18]);
	calib.dig_p8 = (int16_t) BME280_CONCAT_BYTES(calib_data[21],
			calib_data[20]);
	calib.dig_p9 = (int16_t) BME280_CONCAT_BYTES(calib_data[23],
			calib_data[22]);
	calib.dig_h1 = calib_data[25];

	/* Read the humidity calibration data from the sensor */
	reg_addr = BME280_REG_HUMIDITY_CALIB_DATA;
	SMBus::read(reg_addr, BME280_LEN_HUMIDITY_CALIB_DATA, calib_data);
	calib.dig_h2 = (int16_t) BME280_CONCAT_BYTES(calib_data[1], calib_data[0]);
	calib.dig_h3 = calib_data[2];
	calib.dig_h4 = ((int16_t) (int8_t) calib_data[3] * 16)
			| ((int16_t) (calib_data[4] & 0x0F));
	calib.dig_h5 = ((int16_t) (int8_t) calib_data[5] * 16)
			| ((int16_t) (calib_data[4] >> 4));
	calib.dig_h6 = (int8_t) calib_data[6];
}

void BME280::sleep() const {
	uint8_t reg_data[4];
	BME280Config config;

	SMBus::read(BME280_REG_CTRL_HUM, 4, reg_data);
	parseConfiguration(reg_data, config);
	reset();
	reload(config);
}

void BME280::writePowerMode(unsigned char mode) const {
	uint8_t reg_addr = BME280_REG_PWR_CTRL;

	/* Variable to store the value read from power mode register */
	uint8_t sensor_mode_reg_val;

	/* Read the power mode register */
	sensor_mode_reg_val = SMBus::readByte(reg_addr);

	/* Set the power mode */
	sensor_mode_reg_val = BME280_SET_BITS_POS_0(sensor_mode_reg_val,
			BME280_SENSOR_MODE, mode);

	/* Write the power mode in the register */
	SMBus::write(reg_addr, sensor_mode_reg_val);
}

void BME280::parseConfiguration(const unsigned char *data,
		BME280Config &conf) const noexcept {
	if (data) {
		conf.os.humidity = BME280_GET_BITS_POS_0(data[0], BME280_CTRL_HUM);
		conf.os.pressure = BME280_GET_BITS(data[2], BME280_CTRL_PRESS);
		conf.os.temperature = BME280_GET_BITS(data[2], BME280_CTRL_TEMP);
		conf.filter = BME280_GET_BITS(data[3], BME280_FILTER);
		conf.standby = BME280_GET_BITS(data[3], BME280_STANDBY);
	}
}

void BME280::parseRawData(const unsigned char *data) noexcept {
	if (!data) {
		return;
	}

	/* Variables to store the sensor data */
	uint32_t data_xlsb;
	uint32_t data_lsb;
	uint32_t data_msb;

	/* Store the parsed register values for pressure data */
	data_msb = (uint32_t) data[0] << BME280_12_BIT_SHIFT;
	data_lsb = (uint32_t) data[1] << BME280_4_BIT_SHIFT;
	data_xlsb = (uint32_t) data[2] >> BME280_4_BIT_SHIFT;
	raw.pressure = data_msb | data_lsb | data_xlsb;

	/* Store the parsed register values for temperature data */
	data_msb = (uint32_t) data[3] << BME280_12_BIT_SHIFT;
	data_lsb = (uint32_t) data[4] << BME280_4_BIT_SHIFT;
	data_xlsb = (uint32_t) data[5] >> BME280_4_BIT_SHIFT;
	raw.temperature = data_msb | data_lsb | data_xlsb;

	/* Store the parsed register values for humidity data */
	data_msb = (uint32_t) data[6] << BME280_8_BIT_SHIFT;
	data_lsb = (uint32_t) data[7];
	raw.humidity = data_msb | data_lsb;
}

void BME280::reload(const BME280Config &conf) const {
	setOversampling(SEL_ALL_SETTINGS, conf);
	setFilterAndStandby(SEL_ALL_SETTINGS, conf);
}

void BME280::setOversampling(unsigned char desired,
		const BME280Config &conf) const {
	if (desired & SEL_OSR_HUM) {
		setHumidityOSR(conf);
	}

	if (desired & (SEL_OSR_PRESS | SEL_OSR_TEMP)) {
		setTempPresOSR(desired, conf);
	}
}

void BME280::setFilterAndStandby(unsigned char desired,
		const BME280Config &conf) const {
	uint8_t reg_addr = BME280_REG_CONFIG;
	uint8_t reg_data;

	reg_data = SMBus::readByte(reg_addr);

	if (desired & SEL_FILTER) {
		reg_data = BME280_SET_BITS(reg_data, BME280_FILTER, conf.filter);
	}

	if (desired & SEL_STANDBY) {
		reg_data = BME280_SET_BITS(reg_data, BME280_STANDBY, conf.standby);
	}

	/* Write the oversampling settings in the register */
	SMBus::write(reg_addr, reg_data);
}

void BME280::setHumidityOSR(const BME280Config &conf) const {
	uint8_t ctrl_hum;
	uint8_t ctrl_meas;
	uint8_t reg_addr = BME280_REG_CTRL_HUM;

	ctrl_hum = conf.os.humidity & BME280_CTRL_HUM_MSK;

	/* Write the humidity control value in the register */
	SMBus::write(reg_addr, ctrl_hum);

	/* Humidity related changes will be only effective after a
	 * write operation to ctrl_meas register
	 */
	reg_addr = BME280_REG_CTRL_MEAS;
	ctrl_meas = SMBus::readByte(reg_addr);
	SMBus::write(reg_addr, ctrl_meas);

}
void BME280::setTempPresOSR(unsigned char desired,
		const BME280Config &conf) const {
	uint8_t reg_addr = BME280_REG_CTRL_MEAS;
	uint8_t reg_data;

	reg_data = SMBus::readByte(reg_addr);

	if (desired & SEL_OSR_PRESS) {
		reg_data = BME280_SET_BITS(reg_data, BME280_CTRL_PRESS,
				conf.os.pressure);
	}

	if (desired & SEL_OSR_TEMP) {
		reg_data = BME280_SET_BITS(reg_data, BME280_CTRL_TEMP,
				conf.os.temperature);
	}

	/* Write the oversampling settings in the register */
	SMBus::write(reg_addr, reg_data);
}

int BME280::compensateTemperature() noexcept {
	int32_t var1;
	int32_t var2;
	int32_t temperature;
	int32_t temperature_min = -4000;
	int32_t temperature_max = 8500;

	var1 = (int32_t) ((raw.temperature / 8) - ((int32_t) calib.dig_t1 * 2));
	var1 = (var1 * ((int32_t) calib.dig_t2)) / 2048;
	var2 = (int32_t) ((raw.temperature / 16) - ((int32_t) calib.dig_t1));
	var2 = (((var2 * var2) / 4096) * ((int32_t) calib.dig_t3)) / 16384;
	calib.t_fine = var1 + var2;
	temperature = (calib.t_fine * 5 + 128) / 256;

	if (temperature < temperature_min) {
		temperature = temperature_min;
	} else if (temperature > temperature_max) {
		temperature = temperature_max;
	}

	return temperature;
}

unsigned BME280::compensatePressure() const noexcept {
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	uint32_t var5;
	uint32_t pressure;
	uint32_t pressure_min = 30000;
	uint32_t pressure_max = 110000;

	var1 = (((int32_t) calib.t_fine) / 2) - (int32_t) 64000;
	var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t) calib.dig_p6);
	var2 = var2 + ((var1 * ((int32_t) calib.dig_p5)) * 2);
	var2 = (var2 / 4) + (((int32_t) calib.dig_p4) * 65536);
	var3 = (calib.dig_p3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
	var4 = (((int32_t) calib.dig_p2) * var1) / 2;
	var1 = (var3 + var4) / 262144;
	var1 = (((32768 + var1)) * ((int32_t) calib.dig_p1)) / 32768;

	/* Avoid exception caused by division by zero */
	if (var1) {
		var5 = (uint32_t) ((uint32_t) 1048576) - raw.pressure;
		pressure = ((uint32_t) (var5 - (uint32_t) (var2 / 4096))) * 3125;

		if (pressure < 0x80000000) {
			pressure = (pressure << 1) / ((uint32_t) var1);
		} else {
			pressure = (pressure / (uint32_t) var1) * 2;
		}

		var1 = (((int32_t) calib.dig_p9)
				* ((int32_t) (((pressure / 8) * (pressure / 8)) / 8192)))
				/ 4096;
		var2 = (((int32_t) (pressure / 4)) * ((int32_t) calib.dig_p8)) / 8192;
		pressure = (uint32_t) ((int32_t) pressure
				+ ((var1 + var2 + calib.dig_p7) / 16));

		if (pressure < pressure_min) {
			pressure = pressure_min;
		} else if (pressure > pressure_max) {
			pressure = pressure_max;
		}
	} else {
		pressure = pressure_min;
	}

	return pressure;
}

unsigned BME280::compensateHumidity() const noexcept {
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	int32_t var5;
	uint32_t humidity;
	uint32_t humidity_max = 102400;

	var1 = calib.t_fine - ((int32_t) 76800);
	var2 = (int32_t) (raw.humidity * 16384);
	var3 = (int32_t) (((int32_t) calib.dig_h4) * 1048576);
	var4 = ((int32_t) calib.dig_h5) * var1;
	var5 = (((var2 - var3) - var4) + (int32_t) 16384) / 32768;
	var2 = (var1 * ((int32_t) calib.dig_h6)) / 1024;
	var3 = (var1 * ((int32_t) calib.dig_h3)) / 2048;
	var4 = ((var2 * (var3 + (int32_t) 32768)) / 1024) + (int32_t) 2097152;
	var2 = ((var4 * ((int32_t) calib.dig_h2)) + 8192) / 16384;
	var3 = var5 * var2;
	var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
	var5 = var3 - ((var4 * ((int32_t) calib.dig_h1)) / 16);
	var5 = (var5 < 0 ? 0 : var5);
	var5 = (var5 > 419430400 ? 419430400 : var5);
	humidity = (uint32_t) (var5 / 4096);

	if (humidity > humidity_max) {
		humidity = humidity_max;
	}

	return humidity;
}

} /* namespace wanhive */
