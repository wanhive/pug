/*
 * BME69x.cpp
 *
 * Copyright (C) 2025 Wanhive Systems Private Limited (info@wanhive.com)
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
 * Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
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
 */

#include "BME69x.h"
#include <wanhive/base/common/Exception.h>
#include <wanhive/base/Timer.h>
#include <cstdint>

/* Period between two polls (value can be given by user) */
#ifndef BME69X_PERIOD_POLL
#define BME69X_PERIOD_POLL                        UINT32_C(10000)
#endif

/* Period for a soft reset */
#define BME69X_PERIOD_RESET                       UINT32_C(10000)

/* Soft reset command */
#define BME69X_SOFT_RESET_CMD                     UINT8_C(0xb6)

/* Define the shared heating duration */
#define BME69X_W_DEFINE_SHD_HEATR_DUR             INT8_C(3)

/* Information - only available via bme69x_dev.info_msg */
#define BME69X_I_PARAM_CORR                       UINT8_C(1)

/* Register map addresses in I2C */
/* Register for 3rd group of coefficients */
#define BME69X_REG_COEFF3                         UINT8_C(0x00)

/* 0th Field address*/
#define BME69X_REG_FIELD0                         UINT8_C(0x1d)

/* 0th Current DAC address*/
#define BME69X_REG_IDAC_HEAT0                     UINT8_C(0x50)

/* 0th Res heat address */
#define BME69X_REG_RES_HEAT0                      UINT8_C(0x5a)

/* 0th Gas wait address */
#define BME69X_REG_GAS_WAIT0                      UINT8_C(0x64)

/* Shared heating duration address */
#define BME69X_REG_SHD_HEATR_DUR                  UINT8_C(0x6E)

/* CTRL_GAS_0 address */
#define BME69X_REG_CTRL_GAS_0                     UINT8_C(0x70)

/* CTRL_GAS_1 address */
#define BME69X_REG_CTRL_GAS_1                     UINT8_C(0x71)

/* CTRL_HUM address */
#define BME69X_REG_CTRL_HUM                       UINT8_C(0x72)

/* CTRL_MEAS address */
#define BME69X_REG_CTRL_MEAS                      UINT8_C(0x74)

/* CONFIG address */
#define BME69X_REG_CONFIG                         UINT8_C(0x75)

/* MEM_PAGE address */
#define BME69X_REG_MEM_PAGE                       UINT8_C(0xf3)

/* Unique ID address */
#define BME69X_REG_UNIQUE_ID                      UINT8_C(0x83)

/* Register for 1st group of coefficients */
#define BME69X_REG_COEFF1                         UINT8_C(0x8a)

/* Chip ID address */
#define BME69X_REG_CHIP_ID                        UINT8_C(0xd0)

/* Soft reset address */
#define BME69X_REG_SOFT_RESET                     UINT8_C(0xe0)

/* Register for 2nd group of coefficients */
#define BME69X_REG_COEFF2                         UINT8_C(0xe1)

/* Variant ID Register */
#define BME69X_REG_VARIANT_ID                     UINT8_C(0xF0)

/* Coefficient index macros */

/* Length for all coefficients */
#define BME69X_LEN_COEFF_ALL                      UINT8_C(42)

/* Length for 1st group of coefficients */
#define BME69X_LEN_COEFF1                         UINT8_C(23)

/* Length for 2nd group of coefficients */
#define BME69X_LEN_COEFF2                         UINT8_C(14)

/* Length for 3rd group of coefficients */
#define BME69X_LEN_COEFF3                         UINT8_C(5)

/* Length of the field */
#define BME69X_LEN_FIELD                          UINT8_C(17)

/* Length between two fields */
#define BME69X_LEN_FIELD_OFFSET                   UINT8_C(17)

/* Length of the configuration register */
#define BME69X_LEN_CONFIG                         UINT8_C(5)

/* Length of the interleaved buffer */
#define BME69X_LEN_INTERLEAVE_BUFF                UINT8_C(20)

/* Coefficient index macros */

/* Coefficient T2 LSB position */
#define BME69X_IDX_DTK1_C_LSB                     (0)

/* Coefficient T2 MSB position */
#define BME69X_IDX_DTK1_C_MSB                     (1)

/* Coefficient T3 position */
#define BME69X_IDX_DTK2_C                         (2)

/* Coefficient P1 LSB position */
#define BME69X_IDX_S_C_LSB                        (4)

/* Coefficient P1 MSB position */
#define BME69X_IDX_S_C_MSB                        (5)

/* Coefficient P2 LSB position */
#define BME69X_IDX_TK1S_C_LSB                     (6)

/* Coefficient P2 MSB position */
#define BME69X_IDX_TK1S_C_MSB                     (7)

/* Coefficient P3 position */
#define BME69X_IDX_TK2S_C                         (8)

/* Coefficient TK3S position */
#define BME69X_IDX_TK3S_C                         (9)

/* Coefficient P4 LSB position */
#define BME69X_IDX_O_C_LSB                        (10)

/* Coefficient P4 MSB position */
#define BME69X_IDX_O_C_MSB                        (11)

/* Coefficient P5 LSB position */
#define BME69X_IDX_TK10_C_LSB                     (12)

/* Coefficient P5 MSB position */
#define BME69X_IDX_TK10_C_MSB                     (13)

/* Coefficient P7 position */
#define BME69X_IDX_TK20_C                         (14)

/* Coefficient P6 position */
#define BME69X_IDX_TK30_C                         (15)

/* Coefficient P8 LSB position */
#define BME69X_IDX_NLS_C_LSB                      (18)

/* Coefficient P8 MSB position */
#define BME69X_IDX_NLS_C_MSB                      (19)

/* Coefficient P9 LSB position */
#define BME69X_IDX_TKNLS_C                        (20)

/* Coefficient P10 position */
#define BME69X_IDX_NLS3_C                         (21)

/* Coefficient H2 MSB position */
#define BME69X_IDX_S_H_MSB                        (23)

/* Coefficient H2 LSB position */
#define BME69X_IDX_S_H_LSB                        (24)

/* Coefficient H1 LSB position */
#define BME69X_IDX_O_H_LSB                        (24)

/* Coefficient H1 MSB position */
#define BME69X_IDX_O_H_MSB                        (25)

/* Coefficient H3 position */
#define BME69X_IDX_TK10H_C                        (26)

/* Coefficient H4 position */
#define BME69X_IDX_par_h4                         (27)

/* Coefficient H5 position */
#define BME69X_IDX_par_h3                         (28)

/* Coefficient H6 position */
#define BME69X_IDX_HLIN2_C                        (29)

/* Coefficient H7 position */
#define BME69X_IDX_TKHLIN2_C                      (30)

/* Coefficient T1 LSB position */
#define BME69X_IDX_DO_C_LSB                       (31)

/* Coefficient T1 MSB position */
#define BME69X_IDX_DO_C_MSB                       (32)

/* Coefficient GH2 LSB position */
#define BME69X_IDX_TKR_C_LSB                      (33)

/* Coefficient GH2 MSB position */
#define BME69X_IDX_TKR_C_MSB                      (34)

/* Coefficient GH1 position */
#define BME69X_IDX_RO_C                           (35)

/* Coefficient GH3 position */
#define BME69X_IDX_T_AMB_COMP                     (36)

/* Coefficient res heat value position */
#define BME69X_IDX_RES_HEAT_VAL                   (37)

/* Coefficient res heat range position */
#define BME69X_IDX_RES_HEAT_RANGE                 (39)

/* Coefficient range switching error position */
#define BME69X_IDX_RANGE_SW_ERR                   (41)

/* Gas measurement macros */

/* / * BME690 Heater Off Bit Position * / */
#define BME69X_HCTRL                              (3)

/* / * BME690 Run Gas Bit Position * / */
#define BME69X_RUN_GAS                            (6)

/* / * BME690 nb_Conv Bit Position * / */
#define BME69X_NBCONV                             (3)

/* Disable gas measurement */
#define BME69X_DISABLE_GAS_MEAS                   UINT8_C(0x00)

/* Enable gas measurement */
#define BME69X_ENABLE_GAS_MEAS                    UINT8_C(0x01)

/* Heater control macros */

/* Enable heater */
#define BME69X_ENABLE_HEATER                      UINT8_C(0x00)

/* Disable heater */
#define BME69X_DISABLE_HEATER                     UINT8_C(0x01)

#define BME69X_HEATR_DUR1                         UINT16_C(1000)
#define BME69X_HEATR_DUR2                         UINT16_C(2000)
#define BME69X_HEATR_DUR1_DELAY                   UINT32_C(1000000)
#define BME69X_HEATR_DUR2_DELAY                   UINT32_C(2000000)
#define BME69X_N_MEAS                             UINT8_C(6)
#define BME69X_LOW_TEMP                           UINT8_C(150)
#define BME69X_HIGH_TEMP                          UINT16_C(350)

/* Mask macros */
/* Mask for number of conversions */
#define BME69X_NBCONV_MSK                         UINT8_C(0X0f)

/* Mask for IIR filter */
#define BME69X_FILTER_MSK                         UINT8_C(0X1c)

/* Mask for ODR[3] */
#define BME69X_ODR3_MSK                           UINT8_C(0x80)

/* Mask for ODR[2:0] */
#define BME69X_ODR20_MSK                          UINT8_C(0xe0)

/* Mask for temperature oversampling */
#define BME69X_OST_MSK                            UINT8_C(0Xe0)

/* Mask for pressure oversampling */
#define BME69X_OSP_MSK                            UINT8_C(0X1c)

/* Mask for humidity oversampling */
#define BME69X_OSH_MSK                            UINT8_C(0X07)

/* Mask for heater control */
#define BME69X_HCTRL_MSK                          UINT8_C(0x08)

/* Mask for run gas */
#define BME69X_RUN_GAS_MSK                        UINT8_C(0x30)

/* Mask for operation mode */
#define BME69X_MODE_MSK                           UINT8_C(0x03)

/* Mask for res heat range */
#define BME69X_RHRANGE_MSK                        UINT8_C(0x30)

/* Mask for range switching error */
#define BME69X_RSERROR_MSK                        UINT8_C(0xf0)

/* Mask for new data */
#define BME69X_NEW_DATA_MSK                       UINT8_C(0x80)

/* Mask for gas index */
#define BME69X_GAS_INDEX_MSK                      UINT8_C(0x0f)

/* Mask for gas range */
#define BME69X_GAS_RANGE_MSK                      UINT8_C(0x0f)

/* Mask for gas measurement valid */
#define BME69X_GASM_VALID_MSK                     UINT8_C(0x20)

/* Mask for heater stability */
#define BME69X_HEAT_STAB_MSK                      UINT8_C(0x10)

/* Mask for SPI memory page */
#define BME69X_MEM_PAGE_MSK                       UINT8_C(0x10)

/* Mask for reading a register in SPI */
#define BME69X_SPI_RD_MSK                         UINT8_C(0x80)

/* Mask for writing a register in SPI */
#define BME69X_SPI_WR_MSK                         UINT8_C(0x7f)

/* Mask for the H1 calibration coefficient */
#define BME69X_BIT_H1_DATA_MSK                    UINT8_C(0x0f)

/* Position macros */

/* Filter bit position */
#define BME69X_FILTER_POS                         UINT8_C(2)

/* Temperature oversampling bit position */
#define BME69X_OST_POS                            UINT8_C(5)

/* Pressure oversampling bit position */
#define BME69X_OSP_POS                            UINT8_C(2)

/* ODR[3] bit position */
#define BME69X_ODR3_POS                           UINT8_C(7)

/* ODR[2:0] bit position */
#define BME69X_ODR20_POS                          UINT8_C(5)

/* Run gas bit position */
#define BME69X_RUN_GAS_POS                        UINT8_C(5)

/* Heater control bit position */
#define BME69X_HCTRL_POS                          UINT8_C(3)

/* Macro to combine two 8 bit data's to form a 16 bit data */
#define BME69X_CONCAT_BYTES(msb, lsb)             (((uint16_t)msb << 8) | (uint16_t)lsb)

/* Macro to set bits */
#define BME69X_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     ((data << bitname##_POS) & bitname##_MSK))

/* Macro to get bits */
#define BME69X_GET_BITS(reg_data, bitname)        ((reg_data & (bitname##_MSK)) >> \
                                                   (bitname##_POS))

/* Macro to set bits starting from position 0 */
#define BME69X_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     (data & bitname##_MSK))

/* Macro to get bits starting from position 0 */
#define BME69X_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))

namespace {

void swapFields(unsigned index1, unsigned index2,
		wanhive::BME69xData *field[]) noexcept {
	auto temp = field[index1];
	field[index1] = field[index2];
	field[index2] = temp;
}

}  // namespace

namespace wanhive {

BME69x::BME69x(unsigned int bus, unsigned int address) :
		SMBus { bus, address } {
	setup();
}

BME69x::BME69x(const char *path, unsigned int address) :
		SMBus { path, address } {
	setup();
}

BME69x::~BME69x() {

}

void BME69x::setup() {
	reset();
	dev.chipId = SMBus::readByte(BME69X_REG_CHIP_ID);
	if (dev.chipId == CHIP_ID) {
		/* Read Variant ID */
		dev.variantId = SMBus::readByte(BME69X_REG_VARIANT_ID);
		/* Get the Calibration data */
		calibrate();
	} else {
		throw Exception(EX_RESOURCE);
	}
}

void BME69x::reset() const {
	uint8_t reg_addr = BME69X_REG_SOFT_RESET;
	/* 0xb6 is the soft reset command */
	uint8_t soft_rst_cmd = BME69X_SOFT_RESET_CMD;
	/* Reset the device */
	SMBus::write(reg_addr, soft_rst_cmd);
	/* Wait for 5ms */
	Timer::sleep(BME69X_PERIOD_RESET / 1000);
}

void BME69x::setOperationMode(BME69XMode mode) const {
	uint8_t tmp_pow_mode;
	uint8_t pow_mode = 0;
	uint8_t reg_addr = BME69X_REG_CTRL_MEAS;

	/* Call until in sleep */
	do {
		tmp_pow_mode = SMBus::readByte(BME69X_REG_CTRL_MEAS);
		/* Put to sleep before changing mode */
		pow_mode = (tmp_pow_mode & BME69X_MODE_MSK);
		if (pow_mode != BME69X_MODE_SLEEP) {
			tmp_pow_mode &= ~BME69X_MODE_MSK; /* Set to sleep */
			SMBus::write(reg_addr, tmp_pow_mode);
			Timer::sleep(BME69X_PERIOD_POLL / 1000);
		}
	} while (pow_mode != BME69X_MODE_SLEEP);

	/* Already in sleep */
	if (mode != BME69X_MODE_SLEEP) {
		tmp_pow_mode = (tmp_pow_mode & ~BME69X_MODE_MSK)
				| (mode & BME69X_MODE_MSK);
		SMBus::write(reg_addr, tmp_pow_mode);
	}
}

BME69XMode BME69x::getOperationMode() const {
	auto mode = SMBus::readByte(BME69X_REG_CTRL_MEAS);
	return static_cast<BME69XMode>(mode & BME69X_MODE_MSK);
}

unsigned int BME69x::getMeasurementDuration(unsigned char mode,
		const BME69xConfig &conf) const noexcept {
	uint32_t meas_dur = 0; /* Calculate in us */
	uint32_t meas_cycles;
	uint8_t os_to_meas_cycles[6] = { 0, 1, 2, 4, 8, 16 };

	meas_cycles = os_to_meas_cycles[conf.osr.temperature];
	meas_cycles += os_to_meas_cycles[conf.osr.pressure];
	meas_cycles += os_to_meas_cycles[conf.osr.humidity];

	/* TPH measurement duration */
	meas_dur = meas_cycles * UINT32_C(1963);
	meas_dur += UINT32_C(477 * 4); /* TPH switching duration */
	meas_dur += UINT32_C(477 * 5); /* Gas measurement duration */

	if (mode != BME69X_MODE_PARALLEL) {
		meas_dur += UINT32_C(1000); /* Wake up duration of 1ms */
	}

	return meas_dur;
}

void BME69x::setConfiguration(const BME69xConfig &conf) const {
	uint8_t odr20 = 0, odr3 = 1;

	/* Register data starting from BME69X_REG_CTRL_GAS_1(0x71) up to BME69X_REG_CONFIG(0x75) */
	uint8_t reg_array[BME69X_LEN_CONFIG] = { 0x71, 0x72, 0x73, 0x74, 0x75 };
	uint8_t data_array[BME69X_LEN_CONFIG] = { 0 };

	auto current_op_mode = getOperationMode();
	/* Configure only in the sleep mode */
	setOperationMode(BME69X_MODE_SLEEP);

	/* Read the whole configuration and write it back once later */
	SMBus::read(reg_array[0], BME69X_LEN_CONFIG, data_array);
	//rslt = bme69x_get_regs(reg_array[0], data_array, BME69X_LEN_CONFIG, dev);
	//dev->info_msg = BME69X_OK;

	data_array[4] = BME69X_SET_BITS(data_array[4], BME69X_FILTER, conf.filter);
	data_array[3] = BME69X_SET_BITS(data_array[3], BME69X_OST,
			conf.osr.temperature);
	data_array[3] = BME69X_SET_BITS(data_array[3], BME69X_OSP,
			conf.osr.pressure);
	data_array[1] = BME69X_SET_BITS_POS_0(data_array[1], BME69X_OSH,
			conf.osr.humidity);
	if (conf.standby != BME69X_SB_NONE) {
		odr20 = conf.standby;
		odr3 = 0;
	}

	data_array[4] = BME69X_SET_BITS(data_array[4], BME69X_ODR20, odr20);
	data_array[0] = BME69X_SET_BITS(data_array[0], BME69X_ODR3, odr3);

	writeRegisters(reg_array, data_array, BME69X_LEN_CONFIG);
	if ((current_op_mode != BME69X_MODE_SLEEP)) {
		setOperationMode(current_op_mode);
	}
}

void BME69x::getConfiguration(BME69xConfig &conf) const {
	//int8_t rslt;

	/* starting address of the register array for burst read*/
	uint8_t reg_addr = BME69X_REG_CTRL_GAS_1;
	uint8_t data_array[BME69X_LEN_CONFIG];

	SMBus::read(reg_addr, BME69X_LEN_CONFIG, data_array);
	conf.osr.humidity = static_cast<BME69XOverSampling>(BME69X_GET_BITS_POS_0(
			data_array[1], BME69X_OSH));
	conf.osr.temperature = static_cast<BME69XOverSampling>(BME69X_GET_BITS(
			data_array[3], BME69X_OST));
	conf.osr.pressure = static_cast<BME69XOverSampling>(BME69X_GET_BITS(
			data_array[3], BME69X_OSP));
	conf.filter = static_cast<BME69XFilter>(BME69X_GET_BITS(data_array[4],
			BME69X_FILTER));
	if (BME69X_GET_BITS(data_array[0], BME69X_ODR3)) {
		conf.standby = BME69X_SB_NONE;
	} else {
		conf.standby = static_cast<BME69XStandBy>(BME69X_GET_BITS(data_array[4],
				BME69X_ODR20));
	}
}

void BME69x::setHeaterConfiguration(unsigned char mode,
		const BME69xHeaterConfig &conf) const {
	//int8_t rslt;
	uint8_t nb_conv = 0;
	uint8_t hctrl, run_gas = 0;
	uint8_t ctrl_gas_data[2];
	uint8_t ctrl_gas_addr[2] = { BME69X_REG_CTRL_GAS_0, BME69X_REG_CTRL_GAS_1 };

	setOperationMode(BME69X_MODE_SLEEP);
	configureHeater(conf, mode, nb_conv);

	SMBus::read(BME69X_REG_CTRL_GAS_0, 2, ctrl_gas_data);
	if (conf.enable) {
		hctrl = BME69X_ENABLE_HEATER;
		run_gas = BME69X_ENABLE_GAS_MEAS;

	} else {
		hctrl = BME69X_DISABLE_HEATER;
		run_gas = BME69X_DISABLE_GAS_MEAS;
	}

	ctrl_gas_data[0] = BME69X_SET_BITS(ctrl_gas_data[0], BME69X_HCTRL, hctrl);
	ctrl_gas_data[1] = BME69X_SET_BITS_POS_0(ctrl_gas_data[1], BME69X_NBCONV,
			nb_conv);
	ctrl_gas_data[1] = BME69X_SET_BITS(ctrl_gas_data[1], BME69X_RUN_GAS,
			run_gas);

	writeRegisters(ctrl_gas_addr, ctrl_gas_data, 2);
}

void BME69x::getHeaterConfiguration(BME69xHeaterConfig &conf) const {
	//int8_t rslt = BME69X_OK;
	uint8_t data_array[10] = { 0 };
	uint8_t i;

	if ((conf.profile.duration != nullptr)
			&& (conf.profile.temperature != nullptr)
			&& (conf.profile.length <= 10)) {
		/* FIXME: Add conversion to deg C and ms and add the other parameters */
		SMBus::read(BME69X_REG_RES_HEAT0, 10, data_array);
		for (i = 0; i < conf.profile.length; i++) {
			conf.profile.temperature[i] = data_array[i];
		}

		SMBus::read(BME69X_REG_GAS_WAIT0, 10, data_array);

		for (i = 0; i < conf.profile.length; i++) {
			conf.profile.duration[i] = data_array[i];
		}
	} else {
		throw Exception(EX_ARGUMENT);
	}
}

void BME69x::setAmbientTemperature(char temperature) noexcept {
	dev.temperature = temperature;
}

bool BME69x::getData(BME69xData &data) const {
	readFieldData(0, data);
	if (data.status & BME69X_NEW_DATA_MSK) {
		return true;
	} else {
		return false;
	}
}

unsigned int BME69x::getData(BME69xData (&data)[3]) const {
	uint8_t i = 0, j = 0, new_fields = 0;
	BME69xData *field_ptr[3] = { 0 };
	BME69xData field_data[3] = { { 0 } };

	field_ptr[0] = &field_data[0];
	field_ptr[1] = &field_data[1];
	field_ptr[2] = &field_data[2];

	/* Read the 3 fields and count the number of new data fields */
	readAllFieldData(field_ptr);

	new_fields = 0;
	for (i = 0; (i < 3); i++) {
		if (field_ptr[i]->status & BME69X_NEW_DATA_MSK) {
			new_fields++;
		}
	}

	/* Sort the sensor data in parallel & sequential modes*/
	for (i = 0; (i < 2); i++) {
		for (j = i + 1; j < 3; j++) {
			sortSensorData(i, j, field_ptr);
		}
	}

	/* Copy the sorted data */
	for (i = 0; (i < 3); i++) {
		data[i] = *field_ptr[i];
	}

	return new_fields;
}

void BME69x::readFieldData(unsigned char index, BME69xData &data) const {
	//int8_t rslt = BME69X_OK;
	uint8_t buff[BME69X_LEN_FIELD] = { 0 };
	uint8_t gas_range;
	uint32_t adc_temp;
	uint32_t adc_pres;
	volatile uint16_t adc_hum;
	uint16_t adc_gas_res;
	uint8_t tries = 5;

	while ((tries)) {
		SMBus::read(((BME69X_REG_FIELD0 + (index * BME69X_LEN_FIELD_OFFSET))),
		BME69X_LEN_FIELD, buff);

		data.status = buff[0] & BME69X_NEW_DATA_MSK;
		data.gasIndex = buff[0] & BME69X_GAS_INDEX_MSK;
		data.measurementIndex = buff[1];

		/* read the raw data from the sensor */
		adc_pres = (uint32_t) (((uint32_t) buff[2] << 16)
				| ((uint32_t) buff[3] << 8) | ((uint32_t) buff[4]));
		adc_temp = (uint32_t) (((uint32_t) buff[5] << 16)
				| ((uint32_t) buff[6] << 8) | ((uint32_t) buff[7]));
		adc_hum = (uint16_t) (((uint32_t) buff[8] << 8) | (uint32_t) buff[9]);
		adc_gas_res = ((uint16_t) buff[15] << 2) | ((uint16_t) buff[16] >> 6);

		gas_range = buff[16] & BME69X_GAS_RANGE_MSK;

		data.status |= buff[16] & BME69X_GASM_VALID_MSK;
		data.status |= buff[16] & BME69X_HEAT_STAB_MSK;

		if ((data.status & BME69X_NEW_DATA_MSK)) {
			data.heaterResistance = SMBus::readByte(
			BME69X_REG_RES_HEAT0 + data.gasIndex);
			data.idac = SMBus::readByte(BME69X_REG_IDAC_HEAT0 + data.gasIndex);
			data.gasWait = SMBus::readByte(
			BME69X_REG_GAS_WAIT0 + data.gasIndex);
			data.temperature = calcTemperature(adc_temp, data.tCoefficient);
			data.pressure = calcPressure(adc_pres, data.tCoefficient);
			data.humidity = calcHumidity(adc_hum, data.temperature);
			data.gasResistance = calcGasResistance(adc_gas_res, gas_range);
			break;
		}

		Timer::sleep(BME69X_PERIOD_POLL / 1000);
		tries--;
	}
}

void BME69x::readAllFieldData(BME69xData *(&data)[3]) const {
	uint8_t buff[BME69X_LEN_FIELD * 3] = { 0 };
	uint8_t gas_range;
	uint32_t adc_temp;
	uint32_t adc_pres;
	uint16_t adc_hum;
	uint16_t adc_gas_res;
	uint8_t off;
	uint8_t set_val[30] = { 0 }; /* idac, res_heat, gas_wait */
	uint8_t i;

	if (!data[0] && !data[1] && !data[2]) {
		throw Exception(EX_NULL);
	}

	for (unsigned index = 0; index < 3; ++index) {
		auto offset = BME69X_LEN_FIELD * index;
		SMBus::read((BME69X_REG_FIELD0 + offset), BME69X_LEN_FIELD,
				(buff + offset));
	}

	SMBus::read(BME69X_REG_IDAC_HEAT0, 30, set_val);

	for (i = 0; (i < 3); i++) {
		off = (uint8_t) (i * BME69X_LEN_FIELD);
		data[i]->status = buff[off] & BME69X_NEW_DATA_MSK;
		data[i]->gasIndex = buff[off] & BME69X_GAS_INDEX_MSK;
		data[i]->measurementIndex = buff[off + 1];

		/* read the raw data from the sensor */
		adc_pres = (uint32_t) (((uint32_t) buff[off + 2] << 16)
				| ((uint32_t) buff[off + 3] << 8) | ((uint32_t) buff[off + 4]));
		adc_temp = (uint32_t) (((uint32_t) buff[off + 5] << 16)
				| ((uint32_t) buff[off + 6] << 8) | ((uint32_t) buff[off + 7]));
		adc_hum = (uint16_t) (((uint32_t) buff[off + 8] * 256)
				| (uint32_t) buff[off + 9]);
		adc_gas_res = ((uint16_t) buff[off + 15] << 2)
				| ((uint16_t) buff[off + 16] >> 6);
		gas_range = buff[off + 16] & BME69X_GAS_RANGE_MSK;

		data[i]->status |= buff[off + 16] & BME69X_GASM_VALID_MSK;
		data[i]->status |= buff[off + 16] & BME69X_HEAT_STAB_MSK;

		data[i]->idac = set_val[data[i]->gasIndex];
		data[i]->heaterResistance = set_val[10 + data[i]->gasIndex];
		data[i]->gasWait = set_val[20 + data[i]->gasIndex];
		/*
		 * Fixed point calculation needs t_lin for pressure calculation
		 * t_lin is calculated during temperature calculation
		 */
		data[i]->temperature = calcTemperature(adc_temp, data[i]->tCoefficient);
		data[i]->pressure = calcPressure(adc_pres, data[i]->tCoefficient);
		data[i]->humidity = calcHumidity(adc_hum, data[i]->temperature);
		data[i]->gasResistance = calcGasResistance(adc_gas_res, gas_range);
	}
}

void BME69x::sortSensorData(unsigned lowIndex, unsigned highIndex,
		BME69xData *field[]) const noexcept {
	int16_t meas_index1;
	int16_t meas_index2;

	meas_index1 = (int16_t) field[lowIndex]->measurementIndex;
	meas_index2 = (int16_t) field[highIndex]->measurementIndex;
	if ((field[lowIndex]->status & BME69X_NEW_DATA_MSK)
			&& (field[highIndex]->status & BME69X_NEW_DATA_MSK)) {
		int16_t diff = meas_index2 - meas_index1;
		if (((diff > -3) && (diff < 0)) || (diff > 2)) {
			swapFields(lowIndex, highIndex, field);
		}
	} else if (field[highIndex]->status & BME69X_NEW_DATA_MSK) {
		swapFields(lowIndex, highIndex, field);
	}
}

short BME69x::calcTemperature(unsigned int raw,
		unsigned int &tCoeff) const noexcept {
	int64_t partial_data1;
	int64_t partial_data2;
	int64_t partial_data3;
	int64_t partial_data4;
	int64_t partial_data5;
	int64_t partial_data6;
	int64_t tem_comp;

	partial_data1 = (int64_t) (raw - (256U * calib.temp.par_t1));
	partial_data2 = (int64_t) (partial_data1 * (int64_t) calib.temp.par_t2);
	partial_data3 = (int64_t) (partial_data1 * partial_data1);
	partial_data4 = (int64_t) (partial_data3 * (int64_t) calib.temp.par_t3);
	partial_data5 = (int64_t) ((int64_t) (partial_data2 * 262144UL)
			+ partial_data4);
	partial_data6 = (int64_t) (partial_data5 / 4294967296ULL);
	tCoeff = (uint32_t) partial_data6;
	tem_comp = (int64_t) ((partial_data6 * 25U) / 16384UL);

	return (int16_t) (tem_comp);
}

unsigned int BME69x::calcPressure(unsigned int raw,
		unsigned int tCoeff) const noexcept {
	int64_t partial_data1;
	int64_t partial_data2;
	int64_t partial_data3;
	int64_t partial_data4;
	int64_t partial_data5;
	int64_t partial_data6;
	int64_t offset;
	int64_t sensitivity;
	int64_t press_comp;
	int64_t t_lin_64;

	t_lin_64 = (int64_t) tCoeff;

	partial_data1 = t_lin_64 * t_lin_64;
	partial_data2 = partial_data1 / 64;
	partial_data3 = partial_data2 * t_lin_64 / 256;
	partial_data4 = calib.pres.par_p4 * partial_data3 / 32;
	partial_data5 = calib.pres.par_p3 * partial_data1 * 16;
	partial_data6 = calib.pres.par_p2 * t_lin_64 * (1 << 22);

	offset = calib.pres.par_p1 * ((int64_t) 1 << 47) + partial_data4
			+ partial_data5 + partial_data6;
	partial_data2 = (calib.pres.par_p8 * partial_data3) / (1 << 5);
	partial_data4 = calib.pres.par_p7 * partial_data1 * (1 << 2);

	partial_data5 = (calib.pres.par_p6 - 16384) * t_lin_64 * (1 << 21);
	sensitivity = (calib.pres.par_p5 - 16384) * ((int64_t) 1 << 46)
			+ partial_data2 + partial_data4 + partial_data5;
	partial_data1 = sensitivity / (1 << 24) * raw;

	partial_data2 = calib.pres.par_p10 * t_lin_64;
	partial_data3 = partial_data2 + calib.pres.par_p9 * (1 << 16);
	partial_data4 = partial_data3 * raw / (1 << 13);
	partial_data5 = (raw * partial_data4 / 10) / (1 << 9);
	partial_data5 = partial_data5 * 10;
	partial_data6 = raw * raw;

	partial_data2 = calib.pres.par_p11 * partial_data6 / (1 << 16);
	partial_data3 = partial_data2 * raw / (1 << 7);
	partial_data4 = offset / 4 + partial_data1 + partial_data5 + partial_data3;

	press_comp = (partial_data4 / ((int64_t) 1 << 40)) * 25;

	return (uint32_t) (press_comp / 100);
}

unsigned int BME69x::calcHumidity(unsigned short raw,
		short temperature) const noexcept {
	uint32_t hum_comp;
	int64_t hum_64 = raw;
	int64_t t_comp = temperature;
	int64_t t_fine = (t_comp * 256 - 128) / 5;
	int64_t var_H = t_fine - 76800UL;

	var_H = (((((hum_64 * 16384UL) - (calib.hum.par_h1 * 1048576UL)
			- (calib.hum.par_h2 * var_H)) + 16384UL) / 32768UL)
			* ((((((var_H * calib.hum.par_h4) / 1024UL)
					* ((var_H * calib.hum.par_h3) / 2048UL + 32768UL)) / 1024UL)
					+ 2097152ULL) * calib.hum.par_h5 + 8192UL) / 16384UL);

	var_H = var_H
			- (((((var_H / 32768UL) * (var_H / 32768UL)) / 128UL)
					* calib.hum.par_h6) / 16UL);
	hum_comp = (uint32_t) (var_H / 4096UL);

	return hum_comp;
}

unsigned int BME69x::calcGasResistance(unsigned short raw,
		unsigned char gasRange) const noexcept {
	uint32_t calc_gas_res;
	uint32_t var1 = UINT32_C(262144) >> gasRange;
	int32_t var2 = (int32_t) raw - INT32_C(512);

	var2 *= INT32_C(3);
	var2 = INT32_C(4096) + var2;

	/* multiplying 10000 then dividing then multiplying by 100 instead of multiplying by 1000000 to prevent overflow */
	calc_gas_res = (UINT32_C(10000) * var1) / (uint32_t) var2;
	calc_gas_res = calc_gas_res * 100;

	return calc_gas_res;
}

void BME69x::calibrate() {
	uint8_t coeff_array[BME69X_LEN_COEFF_ALL];

	SMBus::read(BME69X_REG_COEFF1, BME69X_LEN_COEFF1, coeff_array);
	SMBus::read(BME69X_REG_COEFF2, BME69X_LEN_COEFF2,
			&coeff_array[BME69X_LEN_COEFF1]);
	SMBus::read(BME69X_REG_COEFF3, BME69X_LEN_COEFF3,
			&coeff_array[BME69X_LEN_COEFF1 + BME69X_LEN_COEFF2]);

	/* Temperature related coefficients */
	calib.temp.par_t1 =
			(uint16_t) (BME69X_CONCAT_BYTES(coeff_array[BME69X_IDX_DO_C_MSB],
					coeff_array[BME69X_IDX_DO_C_LSB]));
	calib.temp.par_t2 = (uint16_t) (BME69X_CONCAT_BYTES(
			coeff_array[BME69X_IDX_DTK1_C_MSB],
			coeff_array[BME69X_IDX_DTK1_C_LSB]));
	calib.temp.par_t3 = (int8_t) (coeff_array[BME69X_IDX_DTK2_C]);

	/* Pressure related coefficients */
	calib.pres.par_p5 = (int16_t) (BME69X_CONCAT_BYTES(
			coeff_array[BME69X_IDX_S_C_MSB], coeff_array[BME69X_IDX_S_C_LSB]));
	calib.pres.par_p6 = (int16_t) (BME69X_CONCAT_BYTES(
			coeff_array[BME69X_IDX_TK1S_C_MSB],
			coeff_array[BME69X_IDX_TK1S_C_LSB]));
	calib.pres.par_p7 = (int8_t) coeff_array[BME69X_IDX_TK2S_C];
	calib.pres.par_p8 = (int8_t) coeff_array[BME69X_IDX_TK3S_C];

	calib.pres.par_p1 = (int16_t) (BME69X_CONCAT_BYTES(
			coeff_array[BME69X_IDX_O_C_MSB], coeff_array[BME69X_IDX_O_C_LSB]));
	calib.pres.par_p2 = (uint16_t) (BME69X_CONCAT_BYTES(
			coeff_array[BME69X_IDX_TK10_C_MSB],
			coeff_array[BME69X_IDX_TK10_C_LSB]));
	calib.pres.par_p3 = (int8_t) (coeff_array[BME69X_IDX_TK20_C]);
	calib.pres.par_p4 = (int8_t) (coeff_array[BME69X_IDX_TK30_C]);

	calib.pres.par_p9 = (int16_t) (BME69X_CONCAT_BYTES(
			coeff_array[BME69X_IDX_NLS_C_MSB],
			coeff_array[BME69X_IDX_NLS_C_LSB]));
	calib.pres.par_p10 = (int8_t) (coeff_array[BME69X_IDX_TKNLS_C]);
	calib.pres.par_p11 = (int8_t) (coeff_array[BME69X_IDX_NLS3_C]);

	/* Humidity related coefficients */
	calib.hum.par_h5 = (int16_t) (((int16_t) coeff_array[BME69X_IDX_S_H_MSB]
			<< 4) | (coeff_array[BME69X_IDX_S_H_LSB] >> 4));
	calib.hum.par_h1 = (int16_t) (((int16_t) coeff_array[BME69X_IDX_O_H_MSB]
			<< 4) | (coeff_array[BME69X_IDX_O_H_LSB] & 0x0F));
	calib.hum.par_h2 = (int8_t) coeff_array[BME69X_IDX_TK10H_C];
	calib.hum.par_h4 = (int8_t) coeff_array[BME69X_IDX_par_h4];
	calib.hum.par_h3 = (uint8_t) coeff_array[BME69X_IDX_par_h3];
	calib.hum.par_h6 = (uint8_t) coeff_array[BME69X_IDX_HLIN2_C];

	/* Gas heater related coefficients */
	calib.gas.par_g1 = (int8_t) coeff_array[BME69X_IDX_RO_C];
	calib.gas.par_g2 = (int16_t) (BME69X_CONCAT_BYTES(
			coeff_array[BME69X_IDX_TKR_C_MSB],
			coeff_array[BME69X_IDX_TKR_C_LSB]));
	calib.gas.par_g3 = (int8_t) coeff_array[BME69X_IDX_T_AMB_COMP];

	/* Other coefficients */
	calib.gas.res_heat_range = ((coeff_array[BME69X_IDX_RES_HEAT_RANGE]
			& BME69X_RHRANGE_MSK) >> 4);
	calib.gas.res_heat_val = (int8_t) coeff_array[BME69X_IDX_RES_HEAT_VAL];
	calib.gas.range_sw_err = ((int8_t) (coeff_array[BME69X_IDX_RANGE_SW_ERR]
			& BME69X_RSERROR_MSK)) / 16;
}

void BME69x::configureHeater(const BME69xHeaterConfig &conf, unsigned char mode,
		unsigned char &nbConv) const {
	uint8_t i;
	uint8_t shared_dur;
	uint8_t write_len = 0;
	uint8_t heater_dur_shared_addr = BME69X_REG_SHD_HEATR_DUR;
	uint8_t rh_reg_addr[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t rh_reg_data[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t gw_reg_addr[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t gw_reg_data[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	switch (mode) {
	case BME69X_MODE_FORCED:
		rh_reg_addr[0] = BME69X_REG_RES_HEAT0;
		rh_reg_data[0] = calculateHeaterResistance(conf.temperature);
		gw_reg_addr[0] = BME69X_REG_GAS_WAIT0;
		gw_reg_data[0] = calculateGasWait(conf.duration);
		nbConv = 0;
		write_len = 1;
		break;
	case BME69X_MODE_SEQUENTIAL:
		if ((!conf.profile.duration) || (!conf.profile.temperature)
				|| (conf.profile.length > 10)) {
			throw Exception(EX_ARGUMENT);
		}

		for (i = 0; i < conf.profile.length; i++) {
			rh_reg_addr[i] = BME69X_REG_RES_HEAT0 + i;
			rh_reg_data[i] = calculateHeaterResistance(
					conf.profile.temperature[i]);
			gw_reg_addr[i] = BME69X_REG_GAS_WAIT0 + i;
			gw_reg_data[i] = calculateGasWait(conf.profile.duration[i]);
		}

		nbConv = conf.profile.length;
		write_len = conf.profile.length;
		break;
	case BME69X_MODE_PARALLEL:
		if ((!conf.profile.duration) || (!conf.profile.temperature)
				|| (conf.profile.length > 10)) {
			throw Exception(EX_ARGUMENT);
		}

		if (conf.profile.sharedDuration == 0) {
			throw Exception(EX_ARGUMENT);
		}

		for (i = 0; i < conf.profile.length; i++) {
			rh_reg_addr[i] = BME69X_REG_RES_HEAT0 + i;
			rh_reg_data[i] = calculateHeaterResistance(
					conf.profile.temperature[i]);
			gw_reg_addr[i] = BME69X_REG_GAS_WAIT0 + i;
			gw_reg_data[i] = (uint8_t) conf.profile.duration[i];
		}

		nbConv = conf.profile.length;
		write_len = conf.profile.length;
		shared_dur = calculateHeaterDurationShared(conf.profile.sharedDuration);
		SMBus::write(heater_dur_shared_addr, shared_dur);
		break;
	default:
		throw Exception(EX_ARGUMENT);
	}

	writeRegisters(rh_reg_addr, rh_reg_data, write_len);
	writeRegisters(gw_reg_addr, gw_reg_data, write_len);
}

unsigned char BME69x::calculateHeaterResistance(
		unsigned short temp) const noexcept {
	uint8_t heatr_res;
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	int32_t var5;
	int32_t heatr_res_x100;

	if (temp > 400) /* Cap temperature */
	{
		temp = 400;
	}

	var1 = (((int32_t) dev.temperature * calib.gas.par_g3) / 1000U) * 256; /* par_g1 */
	var2 = (calib.gas.par_g1 + 784)
			* (((((calib.gas.par_g2 + 154009UL) * temp * 5) / 100) + 3276800ULL)
					/ 10); /* par_g2,
			 * par_g3 */
	var3 = var1 + (var2 >> 1);
	var4 = (var3 / (calib.gas.res_heat_range + 4));
	var5 = (131 * calib.gas.res_heat_val) + 65536UL;
	heatr_res_x100 = (int32_t) (((var4 / var5) - 250) * 34);
	heatr_res = (uint8_t) ((heatr_res_x100 + 50) / 100);

	return heatr_res;
}

unsigned char BME69x::calculateGasWait(unsigned short duration) const noexcept {
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

unsigned char BME69x::calculateHeaterDurationShared(
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

void BME69x::writeRegisters(const unsigned char *commands,
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
