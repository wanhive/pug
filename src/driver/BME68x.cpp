/*
 * BME68x.cpp
 *
 * Copyright (C) 2024 Wanhive Systems Private Limited (info@wanhive.com)
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
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
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

#include "BME68x.h"
#include <wanhive/base/common/Exception.h>
#include <wanhive/base/Timer.h>
#include <cstdint>

/* Period between two polls (value can be given by user) */
#ifndef BME68X_PERIOD_POLL
#define BME68X_PERIOD_POLL                        UINT32_C(10000)
#endif

/* Period for a soft reset */
#define BME68X_PERIOD_RESET                       UINT32_C(10000)

/* Soft reset command */
#define BME68X_SOFT_RESET_CMD                     UINT8_C(0xb6)

/* Return code definitions */
/* Success */
#define BME68X_OK                                 INT8_C(0)

/* Information - only available via bme68x_dev.info_msg */
#define BME68X_I_PARAM_CORR                       UINT8_C(1)

/* Register map addresses in I2C */
/* Register for 3rd group of coefficients */
#define BME68X_REG_COEFF3                         UINT8_C(0x00)

/* 0th Field address*/
#define BME68X_REG_FIELD0                         UINT8_C(0x1d)

/* 0th Current DAC address*/
#define BME68X_REG_IDAC_HEAT0                     UINT8_C(0x50)

/* 0th Res heat address */
#define BME68X_REG_RES_HEAT0                      UINT8_C(0x5a)

/* 0th Gas wait address */
#define BME68X_REG_GAS_WAIT0                      UINT8_C(0x64)

/* Shared heating duration address */
#define BME68X_REG_SHD_HEATR_DUR                  UINT8_C(0x6E)

/* CTRL_GAS_0 address */
#define BME68X_REG_CTRL_GAS_0                     UINT8_C(0x70)

/* CTRL_GAS_1 address */
#define BME68X_REG_CTRL_GAS_1                     UINT8_C(0x71)

/* CTRL_HUM address */
#define BME68X_REG_CTRL_HUM                       UINT8_C(0x72)

/* CTRL_MEAS address */
#define BME68X_REG_CTRL_MEAS                      UINT8_C(0x74)

/* CONFIG address */
#define BME68X_REG_CONFIG                         UINT8_C(0x75)

/* MEM_PAGE address */
#define BME68X_REG_MEM_PAGE                       UINT8_C(0xf3)

/* Unique ID address */
#define BME68X_REG_UNIQUE_ID                      UINT8_C(0x83)

/* Register for 1st group of coefficients */
#define BME68X_REG_COEFF1                         UINT8_C(0x8a)

/* Chip ID address */
#define BME68X_REG_CHIP_ID                        UINT8_C(0xd0)

/* Soft reset address */
#define BME68X_REG_SOFT_RESET                     UINT8_C(0xe0)

/* Register for 2nd group of coefficients */
#define BME68X_REG_COEFF2                         UINT8_C(0xe1)

/* Variant ID Register */
#define BME68X_REG_VARIANT_ID                     UINT8_C(0xF0)

/* SPI page macros */

/* SPI memory page 0 */
#define BME68X_MEM_PAGE0                          UINT8_C(0x10)

/* SPI memory page 1 */
#define BME68X_MEM_PAGE1                          UINT8_C(0x00)

/* Coefficient index macros */

/* Length for all coefficients */
#define BME68X_LEN_COEFF_ALL                      UINT8_C(42)

/* Length for 1st group of coefficients */
#define BME68X_LEN_COEFF1                         UINT8_C(23)

/* Length for 2nd group of coefficients */
#define BME68X_LEN_COEFF2                         UINT8_C(14)

/* Length for 3rd group of coefficients */
#define BME68X_LEN_COEFF3                         UINT8_C(5)

/* Length of the field */
#define BME68X_LEN_FIELD                          UINT8_C(17)

/* Length between two fields */
#define BME68X_LEN_FIELD_OFFSET                   UINT8_C(17)

/* Length of the configuration register */
#define BME68X_LEN_CONFIG                         UINT8_C(5)

/* Length of the interleaved buffer */
#define BME68X_LEN_INTERLEAVE_BUFF                UINT8_C(20)

/* Coefficient index macros */

/* Coefficient T2 LSB position */
#define BME68X_IDX_T2_LSB                         (0)

/* Coefficient T2 MSB position */
#define BME68X_IDX_T2_MSB                         (1)

/* Coefficient T3 position */
#define BME68X_IDX_T3                             (2)

/* Coefficient P1 LSB position */
#define BME68X_IDX_P1_LSB                         (4)

/* Coefficient P1 MSB position */
#define BME68X_IDX_P1_MSB                         (5)

/* Coefficient P2 LSB position */
#define BME68X_IDX_P2_LSB                         (6)

/* Coefficient P2 MSB position */
#define BME68X_IDX_P2_MSB                         (7)

/* Coefficient P3 position */
#define BME68X_IDX_P3                             (8)

/* Coefficient P4 LSB position */
#define BME68X_IDX_P4_LSB                         (10)

/* Coefficient P4 MSB position */
#define BME68X_IDX_P4_MSB                         (11)

/* Coefficient P5 LSB position */
#define BME68X_IDX_P5_LSB                         (12)

/* Coefficient P5 MSB position */
#define BME68X_IDX_P5_MSB                         (13)

/* Coefficient P7 position */
#define BME68X_IDX_P7                             (14)

/* Coefficient P6 position */
#define BME68X_IDX_P6                             (15)

/* Coefficient P8 LSB position */
#define BME68X_IDX_P8_LSB                         (18)

/* Coefficient P8 MSB position */
#define BME68X_IDX_P8_MSB                         (19)

/* Coefficient P9 LSB position */
#define BME68X_IDX_P9_LSB                         (20)

/* Coefficient P9 MSB position */
#define BME68X_IDX_P9_MSB                         (21)

/* Coefficient P10 position */
#define BME68X_IDX_P10                            (22)

/* Coefficient H2 MSB position */
#define BME68X_IDX_H2_MSB                         (23)

/* Coefficient H2 LSB position */
#define BME68X_IDX_H2_LSB                         (24)

/* Coefficient H1 LSB position */
#define BME68X_IDX_H1_LSB                         (24)

/* Coefficient H1 MSB position */
#define BME68X_IDX_H1_MSB                         (25)

/* Coefficient H3 position */
#define BME68X_IDX_H3                             (26)

/* Coefficient H4 position */
#define BME68X_IDX_H4                             (27)

/* Coefficient H5 position */
#define BME68X_IDX_H5                             (28)

/* Coefficient H6 position */
#define BME68X_IDX_H6                             (29)

/* Coefficient H7 position */
#define BME68X_IDX_H7                             (30)

/* Coefficient T1 LSB position */
#define BME68X_IDX_T1_LSB                         (31)

/* Coefficient T1 MSB position */
#define BME68X_IDX_T1_MSB                         (32)

/* Coefficient GH2 LSB position */
#define BME68X_IDX_GH2_LSB                        (33)

/* Coefficient GH2 MSB position */
#define BME68X_IDX_GH2_MSB                        (34)

/* Coefficient GH1 position */
#define BME68X_IDX_GH1                            (35)

/* Coefficient GH3 position */
#define BME68X_IDX_GH3                            (36)

/* Coefficient res heat value position */
#define BME68X_IDX_RES_HEAT_VAL                   (37)

/* Coefficient res heat range position */
#define BME68X_IDX_RES_HEAT_RANGE                 (39)

/* Coefficient range switching error position */
#define BME68X_IDX_RANGE_SW_ERR                   (41)

/* Gas measurement macros */

/* Disable gas measurement */
#define BME68X_DISABLE_GAS_MEAS                   UINT8_C(0x00)

/* Enable gas measurement low */
#define BME68X_ENABLE_GAS_MEAS_L                  UINT8_C(0x01)

/* Enable gas measurement high */
#define BME68X_ENABLE_GAS_MEAS_H                  UINT8_C(0x02)

/* Heater control macros */

/* Enable heater */
#define BME68X_ENABLE_HEATER                      UINT8_C(0x00)

/* Disable heater */
#define BME68X_DISABLE_HEATER                     UINT8_C(0x01)

#define BME68X_HEATR_DUR1                         UINT16_C(1000)
#define BME68X_HEATR_DUR2                         UINT16_C(2000)
#define BME68X_HEATR_DUR1_DELAY                   UINT32_C(1000000)
#define BME68X_HEATR_DUR2_DELAY                   UINT32_C(2000000)
#define BME68X_N_MEAS                             UINT8_C(6)
#define BME68X_LOW_TEMP                           UINT8_C(150)
#define BME68X_HIGH_TEMP                          UINT16_C(350)

/* Mask macros */
/* Mask for number of conversions */
#define BME68X_NBCONV_MSK                         UINT8_C(0X0f)

/* Mask for IIR filter */
#define BME68X_FILTER_MSK                         UINT8_C(0X1c)

/* Mask for ODR[3] */
#define BME68X_ODR3_MSK                           UINT8_C(0x80)

/* Mask for ODR[2:0] */
#define BME68X_ODR20_MSK                          UINT8_C(0xe0)

/* Mask for temperature oversampling */
#define BME68X_OST_MSK                            UINT8_C(0Xe0)

/* Mask for pressure oversampling */
#define BME68X_OSP_MSK                            UINT8_C(0X1c)

/* Mask for humidity oversampling */
#define BME68X_OSH_MSK                            UINT8_C(0X07)

/* Mask for heater control */
#define BME68X_HCTRL_MSK                          UINT8_C(0x08)

/* Mask for run gas */
#define BME68X_RUN_GAS_MSK                        UINT8_C(0x30)

/* Mask for operation mode */
#define BME68X_MODE_MSK                           UINT8_C(0x03)

/* Mask for res heat range */
#define BME68X_RHRANGE_MSK                        UINT8_C(0x30)

/* Mask for range switching error */
#define BME68X_RSERROR_MSK                        UINT8_C(0xf0)

/* Mask for new data */
#define BME68X_NEW_DATA_MSK                       UINT8_C(0x80)

/* Mask for gas index */
#define BME68X_GAS_INDEX_MSK                      UINT8_C(0x0f)

/* Mask for gas range */
#define BME68X_GAS_RANGE_MSK                      UINT8_C(0x0f)

/* Mask for gas measurement valid */
#define BME68X_GASM_VALID_MSK                     UINT8_C(0x20)

/* Mask for heater stability */
#define BME68X_HEAT_STAB_MSK                      UINT8_C(0x10)

/* Mask for SPI memory page */
#define BME68X_MEM_PAGE_MSK                       UINT8_C(0x10)

/* Mask for reading a register in SPI */
#define BME68X_SPI_RD_MSK                         UINT8_C(0x80)

/* Mask for writing a register in SPI */
#define BME68X_SPI_WR_MSK                         UINT8_C(0x7f)

/* Mask for the H1 calibration coefficient */
#define BME68X_BIT_H1_DATA_MSK                    UINT8_C(0x0f)

/* Position macros */

/* Filter bit position */
#define BME68X_FILTER_POS                         UINT8_C(2)

/* Temperature oversampling bit position */
#define BME68X_OST_POS                            UINT8_C(5)

/* Pressure oversampling bit position */
#define BME68X_OSP_POS                            UINT8_C(2)

/* ODR[3] bit position */
#define BME68X_ODR3_POS                           UINT8_C(7)

/* ODR[2:0] bit position */
#define BME68X_ODR20_POS                          UINT8_C(5)

/* Run gas bit position */
#define BME68X_RUN_GAS_POS                        UINT8_C(4)

/* Heater control bit position */
#define BME68X_HCTRL_POS                          UINT8_C(3)

/* Macro to combine two 8 bit data's to form a 16 bit data */
#define BME68X_CONCAT_BYTES(msb, lsb)             (((uint16_t)msb << 8) | (uint16_t)lsb)

/* Macro to set bits */
#define BME68X_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     ((data << bitname##_POS) & bitname##_MSK))

/* Macro to get bits */
#define BME68X_GET_BITS(reg_data, bitname)        ((reg_data & (bitname##_MSK)) >> \
                                                   (bitname##_POS))

/* Macro to set bits starting from position 0 */
#define BME68X_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     (data & bitname##_MSK))

/* Macro to get bits starting from position 0 */
#define BME68X_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))

namespace {

void swapFields(unsigned index1, unsigned index2,
		wanhive::BME68xData *field[]) noexcept {
	wanhive::BME68xData *temp;

	temp = field[index1];
	field[index1] = field[index2];
	field[index2] = temp;
}

}  // namespace

namespace wanhive {

BME68x::BME68x(unsigned int bus, unsigned int address) :
		SMBus(bus, address) {
	setup();
}

BME68x::BME68x(const char *path, unsigned int address) :
		SMBus(path, address) {
	setup();
}

BME68x::~BME68x() {

}

void BME68x::setup() {
	/* Soft reset */
	reset();
	/* Read Chip ID information register */
	dev.chipId = SMBus::readByte(BME68X_REG_CHIP_ID);

	if (dev.chipId == CHIP_ID) {
		/* Read variant ID information register */
		dev.variantId = SMBus::readByte(BME68X_REG_VARIANT_ID);
		dev.ambientTemperature = 25;
		calibrate();
	} else {
		throw Exception(EX_RESOURCE);
	}
}

void BME68x::reset() const {
	SMBus::write(BME68X_REG_SOFT_RESET, (unsigned char) BME68X_SOFT_RESET_CMD);
	/* Wait for 5ms */
	Timer::sleep(BME68X_PERIOD_RESET / 1000);
}

void BME68x::getConfiguration(BME68xConfig &config) const {
	/* starting address of the register array for burst read*/
	uint8_t reg_addr = BME68X_REG_CTRL_GAS_1;
	uint8_t data_array[BME68X_LEN_CONFIG];

	SMBus::read(reg_addr, 5, data_array);
	config.os.humidity = BME68X_GET_BITS_POS_0(data_array[1], BME68X_OSH);
	config.filter = BME68X_GET_BITS(data_array[4], BME68X_FILTER);
	config.os.temperature = BME68X_GET_BITS(data_array[3], BME68X_OST);
	config.os.pressure = BME68X_GET_BITS(data_array[3], BME68X_OSP);
	if (BME68X_GET_BITS(data_array[0], BME68X_ODR3)) {
		config.odr = ODR_NONE;
	} else {
		config.odr = BME68X_GET_BITS(data_array[4], BME68X_ODR20);
	}
}

void BME68x::setConfiguration(const BME68xConfig &config) {
	auto cfg = config;
	uint8_t odr20 = 0, odr3 = 1;
	uint8_t current_op_mode;

	/* Register data starting from BME68X_REG_CTRL_GAS_1(0x71) up to BME68X_REG_CONFIG(0x75) */
	uint8_t reg_array[BME68X_LEN_CONFIG] = { 0x71, 0x72, 0x73, 0x74, 0x75 };
	uint8_t data_array[BME68X_LEN_CONFIG] = { 0 };

	current_op_mode = getOperationMode();
	/* Configure only in the sleep mode */
	setOperationMode(MODE_SLEEP);

	/* Read the whole configuration and write it back once later */
	SMBus::read(reg_array[0], BME68X_LEN_CONFIG, data_array);
	dev.info = BME68X_OK;
	boundaryCheck(cfg.filter, (unsigned char) FILTER_SIZE_127);
	boundaryCheck(cfg.os.temperature, OS_16X);
	boundaryCheck(cfg.os.pressure, OS_16X);
	boundaryCheck(cfg.os.humidity, OS_16X);
	boundaryCheck(cfg.odr, ODR_NONE);

	data_array[4] = BME68X_SET_BITS(data_array[4], BME68X_FILTER, cfg.filter);
	data_array[3] = BME68X_SET_BITS(data_array[3], BME68X_OST,
			cfg.os.temperature);
	data_array[3] = BME68X_SET_BITS(data_array[3], BME68X_OSP, cfg.os.pressure);
	data_array[1] = BME68X_SET_BITS_POS_0(data_array[1], BME68X_OSH,
			cfg.os.humidity);
	if (cfg.odr != ODR_NONE) {
		odr20 = cfg.odr;
		odr3 = 0;
	}

	data_array[4] = BME68X_SET_BITS(data_array[4], BME68X_ODR20, odr20);
	data_array[0] = BME68X_SET_BITS(data_array[0], BME68X_ODR3, odr3);

	writeRegisters(reg_array, data_array, BME68X_LEN_CONFIG);
	if ((current_op_mode != MODE_SLEEP)) {
		setOperationMode(current_op_mode);
	}
}

void BME68x::getHeaterConfiguration(BME68xHeaterConfig &config) const {
	uint8_t data_array[10] = { 0 };
	uint8_t i;

	if ((config.profile.duration != nullptr)
			&& (config.profile.temperature != nullptr)
			&& (config.profile.length <= 10)) {
		/* FIXME: Add conversion to deg C and ms and add the other parameters */
		SMBus::read(BME68X_REG_RES_HEAT0, 10, data_array);

		for (i = 0; i < config.profile.length; i++) {
			config.profile.temperature[i] = data_array[i];
		}

		SMBus::read(BME68X_REG_GAS_WAIT0, 10, data_array);

		for (i = 0; i < config.profile.length; i++) {
			config.profile.duration[i] = data_array[i];
		}
	} else {
		throw Exception(EX_PARAMETER);
	}
}

void BME68x::setHeaterConfiguration(unsigned char opMode,
		const BME68xHeaterConfig &config) {
	uint8_t nb_conv = 0;
	uint8_t hctrl, run_gas = 0;
	uint8_t ctrl_gas_data[2];
	uint8_t ctrl_gas_addr[2] = { BME68X_REG_CTRL_GAS_0, BME68X_REG_CTRL_GAS_1 };

	setOperationMode(MODE_SLEEP);
	configureHeater(config, opMode, nb_conv);

	SMBus::read(BME68X_REG_CTRL_GAS_0, 2, ctrl_gas_data);
	if (config.enable == SW_ENABLE) {
		hctrl = BME68X_ENABLE_HEATER;
		if (dev.variantId == VARIANT_GAS_HIGH) {
			run_gas = BME68X_ENABLE_GAS_MEAS_H;
		} else {
			run_gas = BME68X_ENABLE_GAS_MEAS_L;
		}
	} else {
		hctrl = BME68X_DISABLE_HEATER;
		run_gas = BME68X_DISABLE_GAS_MEAS;
	}

	ctrl_gas_data[0] = BME68X_SET_BITS(ctrl_gas_data[0], BME68X_HCTRL, hctrl);
	ctrl_gas_data[1] = BME68X_SET_BITS_POS_0(ctrl_gas_data[1], BME68X_NBCONV,
			nb_conv);
	ctrl_gas_data[1] = BME68X_SET_BITS(ctrl_gas_data[1], BME68X_RUN_GAS,
			run_gas);
	writeRegisters(ctrl_gas_addr, ctrl_gas_data, 2);
}

unsigned int BME68x::getMeasurementDuration(const unsigned char opMode,
		const BME68xConfig &config) noexcept {
	uint32_t meas_dur = 0; /* Calculate in us */
	uint32_t meas_cycles;
	uint8_t os_to_meas_cycles[6] = { 0, 1, 2, 4, 8, 16 };
	auto conf = config;

	/* Boundary check for temperature oversampling */
	boundaryCheck(conf.os.temperature, OS_16X);
	/* Boundary check for pressure oversampling */
	boundaryCheck(conf.os.pressure, OS_16X);
	/* Boundary check for humidity oversampling */
	boundaryCheck(conf.os.humidity, OS_16X);

	meas_cycles = os_to_meas_cycles[conf.os.temperature];
	meas_cycles += os_to_meas_cycles[conf.os.pressure];
	meas_cycles += os_to_meas_cycles[conf.os.humidity];

	/* TPH measurement duration */
	meas_dur = meas_cycles * UINT32_C(1963);
	meas_dur += UINT32_C(477 * 4); /* TPH switching duration */
	meas_dur += UINT32_C(477 * 5); /* Gas measurement duration */

	if (opMode != MODE_PARALLEL) {
		meas_dur += UINT32_C(1000); /* Wake up duration of 1ms */
	}

	return meas_dur;
}

bool BME68x::getData(BME68xData &data) {
	readFieldData(0, data);
	if (data.status & BME68X_NEW_DATA_MSK) {
		return true;
	} else {
		return false;
	}
}

unsigned int BME68x::getData(BME68xData (&data)[3]) {
	unsigned int new_fields = 0;
	BME68xData *field_ptr[3] = { 0 };
	BME68xData field_data[3] = { { 0 } };

	field_ptr[0] = &field_data[0];
	field_ptr[1] = &field_data[1];
	field_ptr[2] = &field_data[2];

	/* Read the 3 fields and count the number of new data fields */
	readAllFieldData(field_ptr);

	new_fields = 0;
	for (unsigned i = 0; (i < 3); i++) {
		if (field_ptr[i]->status & BME68X_NEW_DATA_MSK) {
			new_fields++;
		}
	}

	/* Sort the sensor data in parallel & sequential modes*/
	for (unsigned i = 0; (i < 2); i++) {
		for (unsigned j = i + 1; j < 3; j++) {
			sortSensorData(i, j, field_ptr);
		}
	}

	/* Copy the sorted data */
	for (unsigned i = 0; (i < 3); i++) {
		data[i] = *field_ptr[i];
	}

	return (new_fields);

}

unsigned char BME68x::getOperationMode() const {

	auto mode = SMBus::readByte(BME68X_REG_CTRL_MEAS);
	return mode & BME68X_MODE_MSK;
}

void BME68x::setOperationMode(unsigned char mode) const {
	uint8_t tmp_pow_mode;
	uint8_t pow_mode = 0;
	uint8_t reg_addr = BME68X_REG_CTRL_MEAS;

	/* Call until in sleep */
	do {
		tmp_pow_mode = SMBus::readByte(BME68X_REG_CTRL_MEAS);
		/* Put to sleep before changing mode */
		pow_mode = (tmp_pow_mode & BME68X_MODE_MSK);
		if (pow_mode != MODE_SLEEP) {
			tmp_pow_mode &= ~BME68X_MODE_MSK; /* Set to sleep */
			SMBus::write(reg_addr, tmp_pow_mode);
			Timer::sleep(BME68X_PERIOD_POLL / 1000);
		}
	} while (pow_mode != MODE_SLEEP);

	/* Already in sleep */
	if ((mode != MODE_SLEEP)) {
		tmp_pow_mode = (tmp_pow_mode & ~BME68X_MODE_MSK)
				| (mode & BME68X_MODE_MSK);
		SMBus::write(reg_addr, tmp_pow_mode);
	}
}

void BME68x::setAmbientTemperature(char temperature) noexcept {
	dev.ambientTemperature = temperature;
}

void BME68x::calibrate() {
	unsigned char coeff_array[BME68X_LEN_COEFF_ALL];

	SMBus::read(BME68X_REG_COEFF1, BME68X_LEN_COEFF1, coeff_array);
	SMBus::read(BME68X_REG_COEFF2, BME68X_LEN_COEFF2,
			&coeff_array[BME68X_LEN_COEFF1]);
	SMBus::read(BME68X_REG_COEFF3, BME68X_LEN_COEFF3,
			&coeff_array[BME68X_LEN_COEFF1 + BME68X_LEN_COEFF2]);

	/* Temperature related coefficients */
	calib.temp.par_t1 = (uint16_t) (BME68X_CONCAT_BYTES(
			coeff_array[BME68X_IDX_T1_MSB], coeff_array[BME68X_IDX_T1_LSB]));
	calib.temp.par_t2 = (int16_t) (BME68X_CONCAT_BYTES(
			coeff_array[BME68X_IDX_T2_MSB], coeff_array[BME68X_IDX_T2_LSB]));
	calib.temp.par_t3 = (int8_t) (coeff_array[BME68X_IDX_T3]);

	/* Pressure related coefficients */
	calib.pres.par_p1 = (uint16_t) (BME68X_CONCAT_BYTES(
			coeff_array[BME68X_IDX_P1_MSB], coeff_array[BME68X_IDX_P1_LSB]));
	calib.pres.par_p2 = (int16_t) (BME68X_CONCAT_BYTES(
			coeff_array[BME68X_IDX_P2_MSB], coeff_array[BME68X_IDX_P2_LSB]));
	calib.pres.par_p3 = (int8_t) coeff_array[BME68X_IDX_P3];
	calib.pres.par_p4 = (int16_t) (BME68X_CONCAT_BYTES(
			coeff_array[BME68X_IDX_P4_MSB], coeff_array[BME68X_IDX_P4_LSB]));
	calib.pres.par_p5 = (int16_t) (BME68X_CONCAT_BYTES(
			coeff_array[BME68X_IDX_P5_MSB], coeff_array[BME68X_IDX_P5_LSB]));
	calib.pres.par_p6 = (int8_t) (coeff_array[BME68X_IDX_P6]);
	calib.pres.par_p7 = (int8_t) (coeff_array[BME68X_IDX_P7]);
	calib.pres.par_p8 = (int16_t) (BME68X_CONCAT_BYTES(
			coeff_array[BME68X_IDX_P8_MSB], coeff_array[BME68X_IDX_P8_LSB]));
	calib.pres.par_p9 = (int16_t) (BME68X_CONCAT_BYTES(
			coeff_array[BME68X_IDX_P9_MSB], coeff_array[BME68X_IDX_P9_LSB]));
	calib.pres.par_p10 = (uint8_t) (coeff_array[BME68X_IDX_P10]);

	/* Humidity related coefficients */
	calib.hum.par_h1 = (uint16_t) (((uint16_t) coeff_array[BME68X_IDX_H1_MSB]
			<< 4) | (coeff_array[BME68X_IDX_H1_LSB] & BME68X_BIT_H1_DATA_MSK));
	calib.hum.par_h2 = (uint16_t) (((uint16_t) coeff_array[BME68X_IDX_H2_MSB]
			<< 4) | ((coeff_array[BME68X_IDX_H2_LSB]) >> 4));
	calib.hum.par_h3 = (int8_t) coeff_array[BME68X_IDX_H3];
	calib.hum.par_h4 = (int8_t) coeff_array[BME68X_IDX_H4];
	calib.hum.par_h5 = (int8_t) coeff_array[BME68X_IDX_H5];
	calib.hum.par_h6 = (uint8_t) coeff_array[BME68X_IDX_H6];
	calib.hum.par_h7 = (int8_t) coeff_array[BME68X_IDX_H7];

	/* Gas heater related coefficients */
	calib.gas.par_g1 = (int8_t) coeff_array[BME68X_IDX_GH1];
	calib.gas.par_g2 = (int16_t) (BME68X_CONCAT_BYTES(
			coeff_array[BME68X_IDX_GH2_MSB], coeff_array[BME68X_IDX_GH2_LSB]));
	calib.gas.par_g3 = (int8_t) coeff_array[BME68X_IDX_GH3];

	/* Other coefficients */
	calib.gas.res_heat_range = ((coeff_array[BME68X_IDX_RES_HEAT_RANGE]
			& BME68X_RHRANGE_MSK) / 16);
	calib.gas.res_heat_val = (int8_t) coeff_array[BME68X_IDX_RES_HEAT_VAL];
	calib.range_sw_err = ((int8_t) (coeff_array[BME68X_IDX_RANGE_SW_ERR]
			& BME68X_RSERROR_MSK)) / 16;
}

void BME68x::configureHeater(const BME68xHeaterConfig &config,
		unsigned char opMode, unsigned char &nConv) const {
	uint8_t i;
	uint8_t shared_dur;
	uint8_t write_len = 0;
	uint8_t heater_dur_shared_addr = BME68X_REG_SHD_HEATR_DUR;
	uint8_t rh_reg_addr[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t rh_reg_data[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t gw_reg_addr[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t gw_reg_data[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	switch (opMode) {
	case MODE_FORCED:
		rh_reg_addr[0] = BME68X_REG_RES_HEAT0;
		rh_reg_data[0] = calculateHeaterResistance(config.temperature);
		gw_reg_addr[0] = BME68X_REG_GAS_WAIT0;
		gw_reg_data[0] = calculateGasWait(config.duration);
		nConv = 0;
		write_len = 1;
		break;
	case MODE_SEQUENTIAL:
		if ((!config.profile.duration) || (!config.profile.temperature)
				|| (config.profile.length > 10)) {
			throw Exception(EX_PARAMETER);
		}

		for (i = 0; i < config.profile.length; i++) {
			rh_reg_addr[i] = BME68X_REG_RES_HEAT0 + i;
			rh_reg_data[i] = calculateHeaterResistance(
					config.profile.temperature[i]);
			gw_reg_addr[i] = BME68X_REG_GAS_WAIT0 + i;
			gw_reg_data[i] = calculateGasWait(config.profile.duration[i]);
		}

		nConv = config.profile.length;
		write_len = config.profile.length;
		break;
	case MODE_PARALLEL:
		if ((!config.profile.duration) || (!config.profile.temperature)
				|| (config.profile.length > 10)) {
			throw Exception(EX_PARAMETER);
		}

		if (config.profile.sharedDuration == 0) {
			throw Exception(EX_PARAMETER);
		}

		for (i = 0; i < config.profile.length; i++) {
			rh_reg_addr[i] = BME68X_REG_RES_HEAT0 + i;
			rh_reg_data[i] = calculateHeaterResistance(
					config.profile.temperature[i]);
			gw_reg_addr[i] = BME68X_REG_GAS_WAIT0 + i;
			gw_reg_data[i] = (uint8_t) config.profile.duration[i];
		}

		nConv = config.profile.length;
		write_len = config.profile.length;
		shared_dur = calculateHeaterDurationShared(
				config.profile.sharedDuration);
		SMBus::write(heater_dur_shared_addr, shared_dur);
		break;
	default:
		throw Exception(EX_PARAMETER);
	}

	writeRegisters(rh_reg_addr, rh_reg_data, write_len);
	writeRegisters(gw_reg_addr, gw_reg_data, write_len);
}

void BME68x::readFieldData(unsigned char index, BME68xData &data) {
	uint8_t buff[BME68X_LEN_FIELD] = { 0 };
	uint8_t gas_range_l, gas_range_h;
	uint32_t adc_temp;
	uint32_t adc_pres;
	uint16_t adc_hum;
	uint16_t adc_gas_res_low, adc_gas_res_high;
	uint8_t tries = 5;

	while (tries) {
		SMBus::read((BME68X_REG_FIELD0 + (index * BME68X_LEN_FIELD_OFFSET)),
		BME68X_LEN_FIELD, buff);

		data.status = buff[0] & BME68X_NEW_DATA_MSK;
		data.gasIndex = buff[0] & BME68X_GAS_INDEX_MSK;
		data.measurementIndex = buff[1];

		/* read the raw data from the sensor */
		adc_pres = (uint32_t) (((uint32_t) buff[2] * 4096)
				| ((uint32_t) buff[3] * 16) | ((uint32_t) buff[4] / 16));
		adc_temp = (uint32_t) (((uint32_t) buff[5] * 4096)
				| ((uint32_t) buff[6] * 16) | ((uint32_t) buff[7] / 16));
		adc_hum = (uint16_t) (((uint32_t) buff[8] * 256) | (uint32_t) buff[9]);
		adc_gas_res_low = (uint16_t) ((uint32_t) buff[13] * 4
				| (((uint32_t) buff[14]) / 64));
		adc_gas_res_high = (uint16_t) ((uint32_t) buff[15] * 4
				| (((uint32_t) buff[16]) / 64));
		gas_range_l = buff[14] & BME68X_GAS_RANGE_MSK;
		gas_range_h = buff[16] & BME68X_GAS_RANGE_MSK;
		if (dev.variantId == VARIANT_GAS_HIGH) {
			data.status |= buff[16] & BME68X_GASM_VALID_MSK;
			data.status |= buff[16] & BME68X_HEAT_STAB_MSK;
		} else {
			data.status |= buff[14] & BME68X_GASM_VALID_MSK;
			data.status |= buff[14] & BME68X_HEAT_STAB_MSK;
		}

		if ((data.status & BME68X_NEW_DATA_MSK)) {
			data.heaterResistance = SMBus::readByte(
			BME68X_REG_RES_HEAT0 + data.gasIndex);
			data.idac = SMBus::readByte(BME68X_REG_IDAC_HEAT0 + data.gasIndex);
			data.gasWait = SMBus::readByte(
			BME68X_REG_GAS_WAIT0 + data.gasIndex);

			data.temperature = calculateTemperature(adc_temp);
			data.pressure = calculatePressure(adc_pres);
			data.humidity = calculateHumidity(adc_hum);
			if (dev.variantId == VARIANT_GAS_HIGH) {
				data.gasResistance = calculateGasResistanceHigh(
						adc_gas_res_high, gas_range_h);
			} else {
				data.gasResistance = calculateGasResistanceLow(adc_gas_res_low,
						gas_range_l);
			}

			break;
		}

		Timer::sleep(BME68X_PERIOD_POLL / 1000);
		tries--;
	}
}

void BME68x::readAllFieldData(BME68xData *(&data)[3]) {
	uint8_t buff[BME68X_LEN_FIELD * 3] = { 0 };
	uint8_t gas_range_l, gas_range_h;
	uint32_t adc_temp;
	uint32_t adc_pres;
	uint16_t adc_hum;
	uint16_t adc_gas_res_low, adc_gas_res_high;
	uint8_t off;
	uint8_t set_val[30] = { 0 }; /* idac, res_heat, gas_wait */

	if (!(data[0] && data[1] && data[2])) {
		throw Exception(EX_NULL);
	}

	for (unsigned index = 0; index < 3; ++index) {
		auto offset = (index * BME68X_LEN_FIELD_OFFSET);
		SMBus::read((BME68X_REG_FIELD0 + offset),
		BME68X_LEN_FIELD, buff + offset);
	}

	SMBus::read(BME68X_REG_IDAC_HEAT0, 30, set_val);

	for (unsigned i = 0; (i < 3); i++) {
		off = (uint8_t) (i * BME68X_LEN_FIELD);
		data[i]->status = buff[off] & BME68X_NEW_DATA_MSK;
		data[i]->gasIndex = buff[off] & BME68X_GAS_INDEX_MSK;
		data[i]->measurementIndex = buff[off + 1];

		/* read the raw data from the sensor */
		adc_pres = (uint32_t) (((uint32_t) buff[off + 2] * 4096)
				| ((uint32_t) buff[off + 3] * 16)
				| ((uint32_t) buff[off + 4] / 16));
		adc_temp = (uint32_t) (((uint32_t) buff[off + 5] * 4096)
				| ((uint32_t) buff[off + 6] * 16)
				| ((uint32_t) buff[off + 7] / 16));
		adc_hum = (uint16_t) (((uint32_t) buff[off + 8] * 256)
				| (uint32_t) buff[off + 9]);
		adc_gas_res_low = (uint16_t) ((uint32_t) buff[off + 13] * 4
				| (((uint32_t) buff[off + 14]) / 64));
		adc_gas_res_high = (uint16_t) ((uint32_t) buff[off + 15] * 4
				| (((uint32_t) buff[off + 16]) / 64));
		gas_range_l = buff[off + 14] & BME68X_GAS_RANGE_MSK;
		gas_range_h = buff[off + 16] & BME68X_GAS_RANGE_MSK;
		if (dev.variantId == VARIANT_GAS_HIGH) {
			data[i]->status |= buff[off + 16] & BME68X_GASM_VALID_MSK;
			data[i]->status |= buff[off + 16] & BME68X_HEAT_STAB_MSK;
		} else {
			data[i]->status |= buff[off + 14] & BME68X_GASM_VALID_MSK;
			data[i]->status |= buff[off + 14] & BME68X_HEAT_STAB_MSK;
		}

		data[i]->idac = set_val[data[i]->gasIndex];
		data[i]->heaterResistance = set_val[10 + data[i]->gasIndex];
		data[i]->gasWait = set_val[20 + data[i]->gasIndex];
		data[i]->temperature = calculateTemperature(adc_temp);
		data[i]->pressure = calculatePressure(adc_pres);
		data[i]->humidity = calculateHumidity(adc_hum);
		if (dev.variantId == VARIANT_GAS_HIGH) {
			data[i]->gasResistance = calculateGasResistanceHigh(
					adc_gas_res_high, gas_range_h);
		} else {
			data[i]->gasResistance = calculateGasResistanceLow(adc_gas_res_low,
					gas_range_l);
		}
	}
}

void BME68x::sortSensorData(unsigned lowIndex, unsigned highIndex,
		BME68xData *field[]) const noexcept {

	int16_t meas_index1;
	int16_t meas_index2;

	meas_index1 = (int16_t) field[lowIndex]->measurementIndex;
	meas_index2 = (int16_t) field[highIndex]->measurementIndex;
	if ((field[lowIndex]->status & BME68X_NEW_DATA_MSK)
			&& (field[highIndex]->status & BME68X_NEW_DATA_MSK)) {
		int16_t diff = meas_index2 - meas_index1;
		if (((diff > -3) && (diff < 0)) || (diff > 2)) {
			swapFields(lowIndex, highIndex, field);
		}
	} else if (field[highIndex]->status & BME68X_NEW_DATA_MSK) {
		swapFields(lowIndex, highIndex, field);
	}
}

void BME68x::boundaryCheck(unsigned char &value, unsigned char max) noexcept {
	/* Check if value is above maximum value */
	if (value > max) {
		/* Auto correct the invalid value to maximum value */
		value = max;
		dev.info |= BME68X_I_PARAM_CORR;
	}
}

short BME68x::calculateTemperature(unsigned int raw) noexcept {
	int64_t var1;
	int64_t var2;
	int64_t var3;
	int16_t calc_temp;

	/*lint -save -e701 -e702 -e704 */
	var1 = ((int32_t) raw >> 3) - ((int32_t) calib.temp.par_t1 << 1);
	var2 = (var1 * (int32_t) calib.temp.par_t2) >> 11;
	var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
	var3 = ((var3) * ((int32_t) calib.temp.par_t3 << 4)) >> 14;
	calib.t_fine = (int32_t) (var2 + var3);
	calc_temp = (int16_t) (((calib.t_fine * 5) + 128) >> 8);

	/*lint -restore */
	return calc_temp;
}

unsigned int BME68x::calculatePressure(unsigned int raw) const noexcept {
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t pressure_comp;

	/* This value is used to check precedence to multiplication or division
	 * in the pressure compensation equation to achieve least loss of precision and
	 * avoiding overflows.
	 * i.e Comparing value, pres_ovf_check = (1 << 31) >> 1
	 */
	const int32_t pres_ovf_check = INT32_C(0x40000000);

	/*lint -save -e701 -e702 -e713 */
	var1 = (((int32_t) calib.t_fine) >> 1) - 64000;
	var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t) calib.pres.par_p6)
			>> 2;
	var2 = var2 + ((var1 * (int32_t) calib.pres.par_p5) << 1);
	var2 = (var2 >> 2) + ((int32_t) calib.pres.par_p4 << 16);
	var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13)
			* ((int32_t) calib.pres.par_p3 << 5)) >> 3)
			+ (((int32_t) calib.pres.par_p2 * var1) >> 1);
	var1 = var1 >> 18;
	var1 = ((32768 + var1) * (int32_t) calib.pres.par_p1) >> 15;
	pressure_comp = 1048576 - raw;
	pressure_comp = (int32_t) ((pressure_comp - (var2 >> 12))
			* ((uint32_t) 3125));
	if (pressure_comp >= pres_ovf_check) {
		pressure_comp = ((pressure_comp / var1) << 1);
	} else {
		pressure_comp = ((pressure_comp << 1) / var1);
	}

	var1 = ((int32_t) calib.pres.par_p9
			* (int32_t) (((pressure_comp >> 3) * (pressure_comp >> 3)) >> 13))
			>> 12;
	var2 = ((int32_t) (pressure_comp >> 2) * (int32_t) calib.pres.par_p8) >> 13;
	var3 = ((int32_t) (pressure_comp >> 8) * (int32_t) (pressure_comp >> 8)
			* (int32_t) (pressure_comp >> 8) * (int32_t) calib.pres.par_p10)
			>> 17;
	pressure_comp = (int32_t) (pressure_comp)
			+ ((var1 + var2 + var3 + ((int32_t) calib.pres.par_p7 << 7)) >> 4);

	/*lint -restore */
	return (uint32_t) pressure_comp;
}

unsigned int BME68x::calculateHumidity(unsigned int raw) const noexcept {
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	int32_t var5;
	int32_t var6;
	int32_t temp_scaled;
	int32_t calc_hum;

	/*lint -save -e702 -e704 */
	temp_scaled = (((int32_t) calib.t_fine * 5) + 128) >> 8;
	var1 = (int32_t) (raw - ((int32_t) ((int32_t) calib.hum.par_h1 * 16)))
			- (((temp_scaled * (int32_t) calib.hum.par_h3) / ((int32_t) 100))
					>> 1);
	var2 = ((int32_t) calib.hum.par_h2
			* (((temp_scaled * (int32_t) calib.hum.par_h4) / ((int32_t) 100))
					+ (((temp_scaled
							* ((temp_scaled * (int32_t) calib.hum.par_h5)
									/ ((int32_t) 100))) >> 6) / ((int32_t) 100))
					+ (int32_t) (1 << 14))) >> 10;
	var3 = var1 * var2;
	var4 = (int32_t) calib.hum.par_h6 << 7;
	var4 = ((var4)
			+ ((temp_scaled * (int32_t) calib.hum.par_h7) / ((int32_t) 100)))
			>> 4;
	var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
	var6 = (var4 * var5) >> 1;
	calc_hum = (((var3 + var6) >> 10) * ((int32_t) 1000)) >> 12;
	if (calc_hum > 100000) /* Cap at 100%rH */
	{
		calc_hum = 100000;
	} else if (calc_hum < 0) {
		calc_hum = 0;
	}

	/*lint -restore */
	return (uint32_t) calc_hum;
}

unsigned int BME68x::calculateGasResistanceLow(unsigned short raw,
		unsigned char range) const noexcept {
	int64_t var1;
	uint64_t var2;
	int64_t var3;
	uint32_t calc_gas_res;
	uint32_t lookup_table1[16] = { UINT32_C(2147483647), UINT32_C(2147483647),
			UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647),
			UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2130303777),
			UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2143188679),
			UINT32_C(2136746228), UINT32_C(2147483647), UINT32_C(2126008810),
			UINT32_C(2147483647), UINT32_C(2147483647) };
	uint32_t lookup_table2[16] = { UINT32_C(4096000000), UINT32_C(2048000000),
			UINT32_C(1024000000), UINT32_C(512000000), UINT32_C(255744255),
			UINT32_C(127110228), UINT32_C(64000000), UINT32_C(32258064),
			UINT32_C(16016016), UINT32_C(8000000), UINT32_C(4000000), UINT32_C(
					2000000), UINT32_C(1000000), UINT32_C(500000), UINT32_C(
					250000), UINT32_C(125000) };

	/*lint -save -e704 */
	var1 = (int64_t) ((1340 + (5 * (int64_t) calib.range_sw_err))
			* ((int64_t) lookup_table1[range])) >> 16;
	var2 = (((int64_t) ((int64_t) raw << 15) - (int64_t) (16777216)) + var1);
	var3 = (((int64_t) lookup_table2[range] * (int64_t) var1) >> 9);
	calc_gas_res = (uint32_t) ((var3 + ((int64_t) var2 >> 1)) / (int64_t) var2);

	/*lint -restore */
	return calc_gas_res;
}

unsigned int BME68x::calculateGasResistanceHigh(unsigned short raw,
		unsigned char range) const noexcept {
	uint32_t calc_gas_res;
	uint32_t var1 = UINT32_C(262144) >> range;
	int32_t var2 = (int32_t) raw - INT32_C(512);

	var2 *= INT32_C(3);
	var2 = INT32_C(4096) + var2;

	/* multiplying 10000 then dividing then multiplying by 100 instead of multiplying by 1000000 to prevent overflow */
	calc_gas_res = (UINT32_C(10000) * var1) / (uint32_t) var2;
	calc_gas_res = calc_gas_res * 100;

	return calc_gas_res;
}

unsigned char BME68x::calculateHeaterResistance(
		unsigned short temperature) const noexcept {
	uint8_t heatr_res;
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	int32_t var5;
	int32_t heatr_res_x100;

	if (temperature > 400) /* Cap temperature */
	{
		temperature = 400;
	}

	var1 = (((int32_t) dev.ambientTemperature * calib.gas.par_g3) / 1000) * 256;
	var2 = (calib.gas.par_g1 + 784)
			* (((((calib.gas.par_g2 + 154009) * temperature * 5) / 100)
					+ 3276800) / 10);
	var3 = var1 + (var2 / 2);
	var4 = (var3 / (calib.gas.res_heat_range + 4));
	var5 = (131 * calib.gas.res_heat_val) + 65536;
	heatr_res_x100 = (int32_t) (((var4 / var5) - 250) * 34);
	heatr_res = (uint8_t) ((heatr_res_x100 + 50) / 100);

	return heatr_res;
}

unsigned char BME68x::calculateGasWait(unsigned short duration) const noexcept {
	uint8_t factor = 0;
	uint8_t durval;

	if (duration >= 0xfc0) {
		durval = 0xff; /* Max duration*/
	} else {
		while (duration > 0x3F) {
			duration = duration / 4;
			factor += 1;
		}

		durval = (uint8_t) (duration + (factor * 64));
	}

	return durval;
}

unsigned char BME68x::calculateHeaterDurationShared(
		unsigned short duration) const noexcept {
	uint8_t factor = 0;
	uint8_t heatdurval;

	if (duration >= 0x783) {
		heatdurval = 0xff; /* Max duration */
	} else {
		/* Step size of 0.477ms */
		duration = (uint16_t) (((uint32_t) duration * 1000) / 477);
		while (duration > 0x3F) {
			duration = duration >> 2;
			factor += 1;
		}

		heatdurval = (uint8_t) (duration + (factor * 64));
	}

	return heatdurval;
}

void BME68x::writeRegisters(const unsigned char *commands,
		const unsigned char *values, unsigned int length) const {
	if (commands && values) {
		for (unsigned int i = 0; i < length; ++i) {
			SMBus::write(commands[i], values[i]);
		}
	} else {
		throw Exception(EX_NULL);
	}
}

} /* namespace wanhive */
