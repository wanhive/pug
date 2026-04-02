/*
 * BMM350.cpp
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
 * Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
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

#include "BMM350.h"
#include <wanhive/base/common/Exception.h>
#include <wanhive/base/Timer.h>
#include <cstdint>
#include <cstring>

/* Macro to SET and GET BITS of a register*/
#define BMM350_SET_BITS(reg_data, bitname, data) \
		((reg_data & ~(bitname##_MSK)) | \
				((data << bitname##_POS) & bitname##_MSK))

#define BMM350_GET_BITS(reg_data, bitname)          ((reg_data & (bitname##_MSK)) >> (bitname##_POS))

#define BMM350_GET_BITS_POS_0(reg_data, bitname)    (reg_data & (bitname##_MSK))

#define BMM350_SET_BITS_POS_0(reg_data, bitname, data) \
		((reg_data & ~(bitname##_MSK)) | \
				(data & bitname##_MSK))

/************************* Sensor Shuttle Variant **************************/
#define BMM350_LEGACY_SHUTTLE_VARIANT_ID            UINT8_C(0x10)
#define BMM350_CURRENT_SHUTTLE_VARIANT_ID           UINT8_C(0x11)

/************************* Sensor delay time settings in microseconds **************************/
#define BMM350_SOFT_RESET_DELAY                     UINT32_C(24000)
#define BMM350_MAGNETIC_RESET_DELAY                 UINT32_C(40000)
#define BMM350_START_UP_TIME_FROM_POR               UINT32_C(3000)

#define BMM350_GOTO_SUSPEND_DELAY                   UINT32_C(6000)
#define BMM350_SUSPEND_TO_NORMAL_DELAY              UINT32_C(38000)

#define BMM350_SUS_TO_FORCEDMODE_NO_AVG_DELAY       UINT32_C(15000)
#define BMM350_SUS_TO_FORCEDMODE_AVG_2_DELAY        UINT32_C(17000)
#define BMM350_SUS_TO_FORCEDMODE_AVG_4_DELAY        UINT32_C(20000)
#define BMM350_SUS_TO_FORCEDMODE_AVG_8_DELAY        UINT32_C(28000)

#define BMM350_SUS_TO_FORCEDMODE_FAST_NO_AVG_DELAY  UINT32_C(4000)
#define BMM350_SUS_TO_FORCEDMODE_FAST_AVG_2_DELAY   UINT32_C(5000)
#define BMM350_SUS_TO_FORCEDMODE_FAST_AVG_4_DELAY   UINT32_C(9000)
#define BMM350_SUS_TO_FORCEDMODE_FAST_AVG_8_DELAY   UINT32_C(16000)

#define BMM350_UPD_OAE_DELAY                        UINT16_C(1000)

#define BMM350_BR_DELAY                             UINT16_C(14000)
#define BMM350_FGR_DELAY                            UINT16_C(18000)

/********************** Register Addresses ************************/

#define BMM350_REG_CHIP_ID                          UINT8_C(0x00)
#define BMM350_REG_ERR_REG                          UINT8_C(0x02)
#define BMM350_REG_PAD_CTRL                         UINT8_C(0x03)
#define BMM350_REG_PMU_CMD_AGGR_SET                 UINT8_C(0x04)
#define BMM350_REG_PMU_CMD_AXIS_EN                  UINT8_C(0x05)
#define BMM350_REG_PMU_CMD                          UINT8_C(0x06)
#define BMM350_REG_PMU_CMD_STATUS_0                 UINT8_C(0x07)
#define BMM350_REG_PMU_CMD_STATUS_1                 UINT8_C(0x08)
#define BMM350_REG_I3C_ERR                          UINT8_C(0x09)
#define BMM350_REG_I2C_WDT_SET                      UINT8_C(0x0A)
#define BMM350_REG_TRSDCR_REV_ID                    UINT8_C(0x0D)
#define BMM350_REG_INT_CTRL                         UINT8_C(0x2E)
#define BMM350_REG_INT_CTRL_IBI                     UINT8_C(0x2F)
#define BMM350_REG_INT_STATUS                       UINT8_C(0x30)
#define BMM350_REG_MAG_X_XLSB                       UINT8_C(0x31)
#define BMM350_REG_MAG_X_LSB                        UINT8_C(0x32)
#define BMM350_REG_MAG_X_MSB                        UINT8_C(0x33)
#define BMM350_REG_MAG_Y_XLSB                       UINT8_C(0x34)
#define BMM350_REG_MAG_Y_LSB                        UINT8_C(0x35)
#define BMM350_REG_MAG_Y_MSB                        UINT8_C(0x36)
#define BMM350_REG_MAG_Z_XLSB                       UINT8_C(0x37)
#define BMM350_REG_MAG_Z_LSB                        UINT8_C(0x38)
#define BMM350_REG_MAG_Z_MSB                        UINT8_C(0x39)
#define BMM350_REG_TEMP_XLSB                        UINT8_C(0x3A)
#define BMM350_REG_TEMP_LSB                         UINT8_C(0x3B)
#define BMM350_REG_TEMP_MSB                         UINT8_C(0x3C)
#define BMM350_REG_SENSORTIME_XLSB                  UINT8_C(0x3D)
#define BMM350_REG_SENSORTIME_LSB                   UINT8_C(0x3E)
#define BMM350_REG_SENSORTIME_MSB                   UINT8_C(0x3F)
#define BMM350_REG_OTP_CMD_REG                      UINT8_C(0x50)
#define BMM350_REG_OTP_DATA_MSB_REG                 UINT8_C(0x52)
#define BMM350_REG_OTP_DATA_LSB_REG                 UINT8_C(0x53)
#define BMM350_REG_OTP_STATUS_REG                   UINT8_C(0x55)
#define BMM350_REG_TMR_SELFTEST_USER                UINT8_C(0x60)
#define BMM350_REG_CTRL_USER                        UINT8_C(0x61)
#define BMM350_REG_CMD                              UINT8_C(0x7E)

/*********************** Macros for bit masking ***************************/

#define BMM350_CHIP_ID_OTP_MSK                      UINT8_C(0xf)
#define BMM350_CHIP_ID_OTP_POS                      UINT8_C(0x0)
#define BMM350_CHIP_ID_FIXED_MSK                    UINT8_C(0xf0)
#define BMM350_CHIP_ID_FIXED_POS                    UINT8_C(0x4)
#define BMM350_PMU_CMD_ERROR_MSK                    UINT8_C(0x1)
#define BMM350_PMU_CMD_ERROR_POS                    UINT8_C(0x0)
#define BMM350_DRV_MSK                              UINT8_C(0x7)
#define BMM350_DRV_POS                              UINT8_C(0x0)
#define BMM350_AVG_MSK                              UINT8_C(0x30)
#define BMM350_AVG_POS                              UINT8_C(0x4)
#define BMM350_ODR_MSK                              UINT8_C(0xf)
#define BMM350_ODR_POS                              UINT8_C(0x0)
#define BMM350_PMU_CMD_MSK                          UINT8_C(0xf)
#define BMM350_PMU_CMD_POS                          UINT8_C(0x0)
#define BMM350_EN_X_MSK                             UINT8_C(0x01)
#define BMM350_EN_X_POS                             UINT8_C(0x0)
#define BMM350_EN_Y_MSK                             UINT8_C(0x02)
#define BMM350_EN_Y_POS                             UINT8_C(0x1)
#define BMM350_EN_Z_MSK                             UINT8_C(0x04)
#define BMM350_EN_Z_POS                             UINT8_C(0x2)
#define BMM350_EN_XYZ_MSK                           UINT8_C(0x7)
#define BMM350_EN_XYZ_POS                           UINT8_C(0x0)
#define BMM350_PMU_CMD_BUSY_MSK                     UINT8_C(0x1)
#define BMM350_PMU_CMD_BUSY_POS                     UINT8_C(0x0)
#define BMM350_ODR_OVWR_MSK                         UINT8_C(0x2)
#define BMM350_ODR_OVWR_POS                         UINT8_C(0x1)
#define BMM350_AVG_OVWR_MSK                         UINT8_C(0x4)
#define BMM350_AVG_OVWR_POS                         UINT8_C(0x2)
#define BMM350_PWR_MODE_IS_NORMAL_MSK               UINT8_C(0x8)
#define BMM350_PWR_MODE_IS_NORMAL_POS               UINT8_C(0x3)
#define BMM350_CMD_IS_ILLEGAL_MSK                   UINT8_C(0x10)
#define BMM350_CMD_IS_ILLEGAL_POS                   UINT8_C(0x4)
#define BMM350_PMU_CMD_VALUE_MSK                    UINT8_C(0xE0)
#define BMM350_PMU_CMD_VALUE_POS                    UINT8_C(0x5)
#define BMM350_PMU_ODR_S_MSK                        UINT8_C(0xf)
#define BMM350_PMU_ODR_S_POS                        UINT8_C(0x0)
#define BMM350_PMU_AVG_S_MSK                        UINT8_C(0x30)
#define BMM350_PMU_AVG_S_POS                        UINT8_C(0x4)
#define BMM350_I3C_ERROR_0_MSK                      UINT8_C(0x1)
#define BMM350_I3C_ERROR_0_POS                      UINT8_C(0x0)
#define BMM350_I3C_ERROR_3_MSK                      UINT8_C(0x8)
#define BMM350_I3C_ERROR_3_POS                      UINT8_C(0x3)
#define BMM350_I2C_WDT_EN_MSK                       UINT8_C(0x1)
#define BMM350_I2C_WDT_EN_POS                       UINT8_C(0x0)
#define BMM350_I2C_WDT_SEL_MSK                      UINT8_C(0x2)
#define BMM350_I2C_WDT_SEL_POS                      UINT8_C(0x1)
#define BMM350_TRSDCR_REV_ID_OTP_MSK                UINT8_C(0x3)
#define BMM350_TRSDCR_REV_ID_OTP_POS                UINT8_C(0x0)
#define BMM350_TRSDCR_REV_ID_FIXED_MSK              UINT8_C(0xfc)
#define BMM350_TRSDCR_REV_ID_FIXED_POS              UINT8_C(0x2)
#define BMM350_DRDY_DATA_REG_MSK                    UINT8_C(0x4)
#define BMM350_DRDY_DATA_REG_POS                    UINT8_C(0x2)
#define BMM350_INT_MODE_MSK                         UINT8_C(0x1)
#define BMM350_INT_MODE_POS                         UINT8_C(0x0)
#define BMM350_INT_POL_MSK                          UINT8_C(0x2)
#define BMM350_INT_POL_POS                          UINT8_C(0x1)
#define BMM350_INT_OD_MSK                           UINT8_C(0x4)
#define BMM350_INT_OD_POS                           UINT8_C(0x2)
#define BMM350_INT_OUTPUT_EN_MSK                    UINT8_C(0x8)
#define BMM350_INT_OUTPUT_EN_POS                    UINT8_C(0x3)
#define BMM350_DRDY_DATA_REG_EN_MSK                 UINT8_C(0x80)
#define BMM350_DRDY_DATA_REG_EN_POS                 UINT8_C(0x7)
#define BMM350_DRDY_INT_MAP_TO_IBI_MSK              UINT8_C(0x1)
#define BMM350_DRDY_INT_MAP_TO_IBI_POS              UINT8_C(0x0)
#define BMM350_CLEAR_DRDY_INT_STATUS_UPON_IBI_MSK   UINT8_C(0x10)
#define BMM350_CLEAR_DRDY_INT_STATUS_UPON_IBI_POS   UINT8_C(0x4)
#define BMM350_CFG_SENS_TIM_AON_MSK                 UINT8_C(0x1)
#define BMM350_CFG_SENS_TIM_AON_POS                 UINT8_C(0x0)
#define BMM350_DATA_X_7_0_MSK                       UINT8_C(0xff)
#define BMM350_DATA_X_7_0_POS                       UINT8_C(0x0)
#define BMM350_DATA_X_15_8_MSK                      UINT8_C(0xff)
#define BMM350_DATA_X_15_8_POS                      UINT8_C(0x0)
#define BMM350_DATA_X_23_16_MSK                     UINT8_C(0xff)
#define BMM350_DATA_X_23_16_POS                     UINT8_C(0x0)
#define BMM350_DATA_Y_7_0_MSK                       UINT8_C(0xff)
#define BMM350_DATA_Y_7_0_POS                       UINT8_C(0x0)
#define BMM350_DATA_Y_15_8_MSK                      UINT8_C(0xff)
#define BMM350_DATA_Y_15_8_POS                      UINT8_C(0x0)
#define BMM350_DATA_Y_23_16_MSK                     UINT8_C(0xff)
#define BMM350_DATA_Y_23_16_POS                     UINT8_C(0x0)
#define BMM350_DATA_Z_7_0_MSK                       UINT8_C(0xff)
#define BMM350_DATA_Z_7_0_POS                       UINT8_C(0x0)
#define BMM350_DATA_Z_15_8_MSK                      UINT8_C(0xff)
#define BMM350_DATA_Z_15_8_POS                      UINT8_C(0x0)
#define BMM350_DATA_Z_23_16_MSK                     UINT8_C(0xff)
#define BMM350_DATA_Z_23_16_POS                     UINT8_C(0x0)
#define BMM350_DATA_T_7_0_MSK                       UINT8_C(0xff)
#define BMM350_DATA_T_7_0_POS                       UINT8_C(0x0)
#define BMM350_DATA_T_15_8_MSK                      UINT8_C(0xff)
#define BMM350_DATA_T_15_8_POS                      UINT8_C(0x0)
#define BMM350_DATA_T_23_16_MSK                     UINT8_C(0xff)
#define BMM350_DATA_T_23_16_POS                     UINT8_C(0x0)
#define BMM350_DATA_ST_7_0_MSK                      UINT8_C(0xff)
#define BMM350_DATA_ST_7_0_POS                      UINT8_C(0x0)
#define BMM350_DATA_ST_15_8_MSK                     UINT8_C(0xff)
#define BMM350_DATA_ST_15_8_POS                     UINT8_C(0x0)
#define BMM350_DATA_ST_23_16_MSK                    UINT8_C(0xff)
#define BMM350_DATA_ST_23_16_POS                    UINT8_C(0x0)

/****************************** OTP MACROS ***************************/
#define BMM350_OTP_CMD_DIR_READ                     UINT8_C(0x20)
#define BMM350_OTP_CMD_DIR_PRGM_1B                  UINT8_C(0x40)
#define BMM350_OTP_CMD_DIR_PRGM                     UINT8_C(0x60)
#define BMM350_OTP_CMD_PWR_OFF_OTP                  UINT8_C(0x80)
#define BMM350_OTP_CMD_EXT_READ                     UINT8_C(0xA0)
#define BMM350_OTP_CMD_EXT_PRGM                     UINT8_C(0xE0)
#define BMM350_OTP_CMD_MSK                          UINT8_C(0xE0)
#define BMM350_OTP_WORD_ADDR_MSK                    UINT8_C(0x1F)

#define BMM350_OTP_STATUS_ERROR_MSK                 UINT8_C(0xE0)
#define BMM350_OTP_STATUS_ERROR(val)                (val & BMM350_OTP_STATUS_ERROR_MSK)
#define BMM350_OTP_STATUS_NO_ERROR                  UINT8_C(0x00)
#define BMM350_OTP_STATUS_BOOT_ERR                  UINT8_C(0x20)
#define BMM350_OTP_STATUS_PAGE_RD_ERR               UINT8_C(0x40)
#define BMM350_OTP_STATUS_PAGE_PRG_ERR              UINT8_C(0x60)
#define BMM350_OTP_STATUS_SIGN_ERR                  UINT8_C(0x80)
#define BMM350_OTP_STATUS_INV_CMD_ERR               UINT8_C(0xA0)
#define BMM350_OTP_STATUS_CMD_DONE                  UINT8_C(0x01)

/****************************** OTP indices ***************************/
#define BMM350_TEMP_OFF_SENS                        UINT8_C(0x0D)

#define BMM350_MAG_OFFSET_X                         UINT8_C(0x0E)
#define BMM350_MAG_OFFSET_Y                         UINT8_C(0x0F)
#define BMM350_MAG_OFFSET_Z                         UINT8_C(0x10)

#define BMM350_MAG_SENS_X                           UINT8_C(0x10)
#define BMM350_MAG_SENS_Y                           UINT8_C(0x11)
#define BMM350_MAG_SENS_Z                           UINT8_C(0x11)

#define BMM350_MAG_TCO_X                            UINT8_C(0x12)
#define BMM350_MAG_TCO_Y                            UINT8_C(0x13)
#define BMM350_MAG_TCO_Z                            UINT8_C(0x14)

#define BMM350_MAG_TCS_X                            UINT8_C(0x12)
#define BMM350_MAG_TCS_Y                            UINT8_C(0x13)
#define BMM350_MAG_TCS_Z                            UINT8_C(0x14)

#define BMM350_MAG_DUT_T_0                          UINT8_C(0x18)

#define BMM350_CROSS_X_Y                            UINT8_C(0x15)
#define BMM350_CROSS_Y_X                            UINT8_C(0x15)
#define BMM350_CROSS_Z_X                            UINT8_C(0x16)
#define BMM350_CROSS_Z_Y                            UINT8_C(0x16)

#define BMM350_SENS_CORR_Y                          (0.01f)
#define BMM350_TCS_CORR_Z                           (0.0001f)

/**************************** Signed bit macros **********************/
#define BMM350_SIGNED_8_BIT                         UINT8_C(8)
#define BMM350_SIGNED_12_BIT                        UINT8_C(12)
#define BMM350_SIGNED_16_BIT                        UINT8_C(16)
#define BMM350_SIGNED_21_BIT                        UINT8_C(21)
#define BMM350_SIGNED_24_BIT                        UINT8_C(24)

/**************************** Self-test macros **********************/
#define BMM350_SELF_TEST_DISABLE                    UINT8_C(0x00)
#define BMM350_SELF_TEST_POS_X                      UINT8_C(0x0D)
#define BMM350_SELF_TEST_NEG_X                      UINT8_C(0x0B)
#define BMM350_SELF_TEST_POS_Y                      UINT8_C(0x15)
#define BMM350_SELF_TEST_NEG_Y                      UINT8_C(0x13)

/**************************** PMU command status 0 macros **********************/
#define BMM350_PMU_CMD_STATUS_0_SUS                 UINT8_C(0x00)
#define BMM350_PMU_CMD_STATUS_0_NM                  UINT8_C(0x01)
#define BMM350_PMU_CMD_STATUS_0_UPD_OAE             UINT8_C(0x02)
#define BMM350_PMU_CMD_STATUS_0_FM                  UINT8_C(0x03)
#define BMM350_PMU_CMD_STATUS_0_FM_FAST             UINT8_C(0x04)
#define BMM350_PMU_CMD_STATUS_0_FGR                 UINT8_C(0x05)
#define BMM350_PMU_CMD_STATUS_0_FGR_FAST            UINT8_C(0x06)
#define BMM350_PMU_CMD_STATUS_0_BR                  UINT8_C(0x07)
#define BMM350_PMU_CMD_STATUS_0_BR_FAST             UINT8_C(0x07)

#define BMM350_LSB_MASK                             UINT16_C(0x00FF)
#define BMM350_MSB_MASK                             UINT16_C(0xFF00)

#define BMM350_DUMMY_BYTES                          UINT8_C(2)
/********************* Power modes *************************/
#define BMM350_PMU_CMD_UPD_OAE                      UINT8_C(0x02)

#define BMM350_PMU_STATUS_0                         UINT8_C(0x0)

#define BMM350_CMD_NOP                              UINT8_C(0x0)
#define BMM350_CMD_SOFTRESET                        UINT8_C(0xB6)

#define BMM350_INT_OUTPUT_EN_OFF                    UINT8_C(0x0)
#define BMM350_INT_OUTPUT_EN_ON                     UINT8_C(0x1)

#define BMM350_INT_DRDY_EN                          UINT8_C(0x1)
#define BMM350_INT_DRDY_DIS                         UINT8_C(0x0)

namespace {

int32_t fix_sign(uint32_t inval, int8_t number_of_bits) {
	int32_t power = 0;
	int32_t retval;

	switch (number_of_bits) {
	case BMM350_SIGNED_8_BIT:
		power = 128; /* 2^7 */
		break;

	case BMM350_SIGNED_12_BIT:
		power = 2048; /* 2^11 */
		break;

	case BMM350_SIGNED_16_BIT:
		power = 32768; /* 2^15 */
		break;

	case BMM350_SIGNED_21_BIT:
		power = 1048576; /* 2^20 */
		break;

	case BMM350_SIGNED_24_BIT:
		power = 8388608; /* 2^23 */
		break;

	default:
		power = 0;
		break;
	}

	retval = (int32_t) inval;

	if (retval >= power) {
		retval = retval - (power * 2);
	}

	return retval;
}

void update_default_coefficients(float (&lsb_to_ut_degc)[4]) {
	float bxy_sens, bz_sens, temp_sens, ina_xy_gain_trgt, ina_z_gain_trgt,
			adc_gain, lut_gain;
	float power;

	bxy_sens = 14.55f;
	bz_sens = 9.0f;
	temp_sens = 0.00204f;

	ina_xy_gain_trgt = 19.46f;

	ina_z_gain_trgt = 31.0;

	adc_gain = 1 / 1.5f;
	lut_gain = 0.714607238769531f;

	power = (float) (1000000.0 / 1048576.0);

	lsb_to_ut_degc[0] = (power
			/ (bxy_sens * ina_xy_gain_trgt * adc_gain * lut_gain));
	lsb_to_ut_degc[1] = (power
			/ (bxy_sens * ina_xy_gain_trgt * adc_gain * lut_gain));
	lsb_to_ut_degc[2] = (power
			/ (bz_sens * ina_z_gain_trgt * adc_gain * lut_gain));
	lsb_to_ut_degc[3] = 1 / (temp_sens * adc_gain * lut_gain * 1048576);
}

}  // namespace

namespace wanhive {

BMM350::BMM350(unsigned int bus, unsigned int address) :
		SMBus(bus, address) {
	setup();
}

BMM350::BMM350(const char *path, unsigned int address) :
		SMBus(path, address) {
	setup();
}

BMM350::~BMM350() {

}

void BMM350::setup() {
	/* Variable to get chip id */
	uint8_t chip_id = BMM350_DISABLE;

	/* Variable to store the command to power-off the OTP */
	uint8_t otp_cmd = BMM350_OTP_CMD_PWR_OFF_OTP;

	/* Variable to store soft-reset command */
	uint8_t soft_reset;

	dev.chip = 0;

	/* Assign axis_en with all axis enabled (BMM350_EN_XYZ_MSK) */
	dev.axes = BMM350_EN_XYZ_MSK;

	Timer::sleep(BMM350_START_UP_TIME_FROM_POR / 1000);

	/* Soft-reset */
	soft_reset = BMM350_CMD_SOFTRESET;
	/* Set the command in the command register */
	SMBus::write(BMM350_REG_CMD, soft_reset);
	Timer::sleep(BMM350_SOFT_RESET_DELAY / 1000);

	/* Chip ID of the sensor is read */
	chip_id = readRegByte(BMM350_REG_CHIP_ID);

	/* Assign chip_id to dev->chip_id */
	dev.chip = chip_id;

	/* Check for chip id validity */
	if ((dev.chip == CHIP_ID)) {
		/* Download OTP memory */
		readOTPData();

		/* Power off OTP */
		SMBus::write(BMM350_REG_OTP_CMD_REG, otp_cmd);

		magneticResetAndWait();
	} else {
		throw Exception(EX_RESOURCE);
	}
}

void BMM350::reset() {
	uint8_t reg_data;

	/* Variable to store the command to power-off the OTP */
	uint8_t otp_cmd = BMM350_OTP_CMD_PWR_OFF_OTP;

	reg_data = BMM350_CMD_SOFTRESET;
	/* Set the command in the command register */
	SMBus::write(BMM350_REG_CMD, reg_data);
	Timer::sleep(BMM350_SOFT_RESET_DELAY / 1000);

	/* Power off OTP */
	SMBus::write(BMM350_REG_OTP_CMD_REG, otp_cmd);
	magneticResetAndWait();
}

unsigned char BMM350::getInterruptStatus() {
	uint8_t int_status_reg;

	/* Get the status of interrupt */
	int_status_reg = readRegByte(BMM350_REG_INT_STATUS);

	/* Read the interrupt status */
	return BMM350_GET_BITS(int_status_reg, BMM350_DRDY_DATA_REG);
}

void BMM350::setPowerMode(BMM350PowerMode mode) {
	uint8_t last_pwr_mode;
	uint8_t reg_data;

	last_pwr_mode = readRegByte(BMM350_REG_PMU_CMD);

	if (last_pwr_mode > BMM350_BITRESET_FAST) {
		throw Exception(EX_STATE);
	}

	if (((last_pwr_mode == BMM350_MODE_NORMAL)
			|| (last_pwr_mode == BMM350_PMU_CMD_UPD_OAE))) {
		reg_data = BMM350_MODE_SUSPEND;

		/* Set PMU command configuration */
		SMBus::write(BMM350_REG_PMU_CMD, reg_data);

		Timer::sleep(BMM350_GOTO_SUSPEND_DELAY / 1000);
	}

	setPowerModeInternal(mode);
}

void BMM350::setPerformance(const BMM350PerformanceConfig &ocfg) {
	/* Variable to get PMU command */
	uint8_t reg_data = 0;

	enum BMM350SamplesAveraging performance_fix = ocfg.averaging;
	/* Reduce the performance setting when too high for the chosen ODR */
	if ((ocfg.dataRate == BMM350_ODR_400HZ)
			&& (ocfg.averaging >= BMM350_AVERAGING_2)) {
		performance_fix = BMM350_AVERAGING_NONE;
	} else if ((ocfg.dataRate == BMM350_ODR_200HZ)
			&& (ocfg.averaging >= BMM350_AVERAGING_4)) {
		performance_fix = BMM350_AVERAGING_2;
	} else if ((ocfg.dataRate == BMM350_ODR_100HZ)
			&& (ocfg.averaging >= BMM350_AVERAGING_8)) {
		performance_fix = BMM350_AVERAGING_4;
	}

	/* ODR is an enum taking the generated constants from the register map */
	reg_data = ((uint8_t) ocfg.dataRate & BMM350_ODR_MSK);

	/* AVG / performance is an enum taking the generated constants from the register map */
	reg_data = BMM350_SET_BITS(reg_data, BMM350_AVG, (uint8_t )performance_fix);

	/* Set PMU command configurations for ODR and performance */
	SMBus::write(BMM350_REG_PMU_CMD_AGGR_SET, reg_data);

	/* Set PMU command configurations to update odr and average */
	reg_data = BMM350_PMU_CMD_UPD_OAE;

	/* Set PMU command configuration */
	SMBus::write(BMM350_REG_PMU_CMD, reg_data);

	Timer::sleep(BMM350_UPD_OAE_DELAY / 1000);
}

void BMM350::setAxes(bool enableX, bool enableY, bool enableZ) {
	/* Variable to store axis data */
	uint8_t data;

	if (!(enableX || enableY || enableZ)) {
		dev.axes = BMM350_DISABLE;
	} else {
		data = ((uint8_t) enableX & BMM350_EN_X_MSK);
		data = BMM350_SET_BITS(data, BMM350_EN_Y, (uint8_t )enableY);
		data = BMM350_SET_BITS(data, BMM350_EN_Z, (uint8_t )enableY);

		SMBus::write(BMM350_REG_PMU_CMD_AXIS_EN, data);
		/* Assign axis_en with the axis selection done */
		dev.axes = data;
	}
}

void BMM350::readSensorTime(unsigned int &seconds, unsigned int &nanoseconds) {
	uint64_t time;

	uint8_t reg_data[3];
	/* Get sensor time raw data */
	readRegBytes(BMM350_REG_SENSORTIME_XLSB, 3, reg_data);

	time = (reg_data[0] + ((uint32_t) reg_data[1] << 8)
			+ ((uint32_t) reg_data[2] << 16));

	/* 1 LSB is 39.0625us. Converting to nanoseconds */
	time *= UINT64_C(390625);
	time /= UINT64_C(10);
	seconds = (uint32_t) (time / UINT64_C(1000000000));
	nanoseconds = (uint32_t) (time - ((seconds) * UINT64_C(1000000000)));
}

void BMM350::setInterrupt(bool enable) {
	/* Variable to get interrupt control configuration */
	uint8_t reg_data = 0;

	/* Get interrupt control configuration */
	reg_data = readRegByte(BMM350_REG_INT_CTRL);

	reg_data = BMM350_SET_BITS(reg_data, BMM350_DRDY_DATA_REG_EN,
			(uint8_t )enable);

	/* Finally transfer the interrupt configurations */
	SMBus::write(BMM350_REG_INT_CTRL, reg_data);
}

void BMM350::configureInterrupt(const BMM350InterruptConfig &icfg) {
	/* Variable to get interrupt control configuration */
	uint8_t reg_data = 0;

	/* Get interrupt control configuration */
	reg_data = readRegByte(BMM350_REG_INT_CTRL);

	reg_data = BMM350_SET_BITS_POS_0(reg_data, BMM350_INT_MODE, icfg.latching);
	reg_data = BMM350_SET_BITS(reg_data, BMM350_INT_POL, icfg.polarity);
	reg_data = BMM350_SET_BITS(reg_data, BMM350_INT_OD, icfg.drivertype);
	reg_data = BMM350_SET_BITS(reg_data, BMM350_INT_OUTPUT_EN,
			(uint8_t )icfg.mapped);

	/* Finally transfer the interrupt configurations */
	SMBus::write(BMM350_REG_INT_CTRL, reg_data);
}

void BMM350::readRawData(BMM350RawData &data) {
	uint8_t mag_data[MAG_TEMP_DATA_LENGTH] { };

	uint32_t raw_mag_x, raw_mag_y, raw_mag_z, raw_temp;

	/* Get uncompensated mag data */
	readRegBytes(BMM350_REG_MAG_X_XLSB, MAG_TEMP_DATA_LENGTH, mag_data);

	raw_mag_x = (uint32_t) mag_data[0] + ((uint32_t) mag_data[1] << 8)
			+ ((uint32_t) mag_data[2] << 16);
	raw_mag_y = (uint32_t) mag_data[3] + ((uint32_t) mag_data[4] << 8)
			+ ((uint32_t) mag_data[5] << 16);
	raw_mag_z = (uint32_t) mag_data[6] + ((uint32_t) mag_data[7] << 8)
			+ ((uint32_t) mag_data[8] << 16);
	raw_temp = (uint32_t) mag_data[9] + ((uint32_t) mag_data[10] << 8)
			+ ((uint32_t) mag_data[11] << 16);

	if ((dev.axes & BMM350_EN_X_MSK) == BMM350_DISABLE) {
		data.x = BMM350_DISABLE;
	} else {
		data.x = fix_sign(raw_mag_x, BMM350_SIGNED_24_BIT);
	}

	if ((dev.axes & BMM350_EN_Y_MSK) == BMM350_DISABLE) {
		data.y = BMM350_DISABLE;
	} else {
		data.y = fix_sign(raw_mag_y, BMM350_SIGNED_24_BIT);
	}

	if ((dev.axes & BMM350_EN_Z_MSK) == BMM350_DISABLE) {
		data.z = BMM350_DISABLE;
	} else {
		data.z = fix_sign(raw_mag_z, BMM350_SIGNED_24_BIT);
	}

	data.temperature = fix_sign(raw_temp, BMM350_SIGNED_24_BIT);
}

void BMM350::setInterruptControlIBI(bool enable, bool clearOnIBI) {
	/* Variable to get interrupt control configuration */
	uint8_t reg_data = 0;

	/* Get interrupt control configuration */
	reg_data = readRegByte(BMM350_REG_INT_CTRL_IBI);

	reg_data = BMM350_SET_BITS_POS_0(reg_data, BMM350_DRDY_INT_MAP_TO_IBI,
			(uint8_t )enable);
	reg_data = BMM350_SET_BITS(reg_data, BMM350_CLEAR_DRDY_INT_STATUS_UPON_IBI,
			(uint8_t )clearOnIBI);

	/* Set the IBI control configuration */
	SMBus::write(BMM350_REG_INT_CTRL_IBI, reg_data);

	if (enable) {
		/* Enable data ready interrupt if IBI is enabled */
		setInterrupt(true);
	}
}

void BMM350::setPadDrive(unsigned char drive) {
	uint8_t reg_data;
	if (drive < PAD_DRIVE_WEAKEST) {
		drive = PAD_DRIVE_WEAKEST;
	} else if (drive > PAD_DRIVE_STRONGEST) {
		drive = PAD_DRIVE_STRONGEST;
	}

	reg_data = drive & BMM350_DRV_MSK;

	/* Set drive */
	SMBus::write(BMM350_REG_PAD_CTRL, reg_data);
}

void BMM350::magneticResetAndWait() {
	uint8_t pmu_cmd = 0;
	struct BMM350PmuCmdStatus0 pmu_cmd_stat_0 { };
	uint8_t restore_normal = BMM350_DISABLE;

	//TODO: magnetic wait and reset override

	/* Read PMU CMD status */
	getPMUCommandStatus0(pmu_cmd_stat_0);

	/* Check the powermode is normal before performing magnetic reset */
	if ((pmu_cmd_stat_0.normalPowerMode == BMM350_ENABLE)) {
		restore_normal = BMM350_ENABLE;

		/* Reset can only be triggered in suspend */
		setPowerMode(BMM350_MODE_SUSPEND);
	}

	/* Set BR to PMU_CMD register */
	pmu_cmd = BMM350_BITRESET_9MS;

	SMBus::write(BMM350_REG_PMU_CMD, pmu_cmd);
	Timer::sleep(BMM350_BR_DELAY / 1000);

	/* Verify if PMU_CMD_STATUS_0 register has BR set */
	getPMUCommandStatus0(pmu_cmd_stat_0);

	if ((pmu_cmd_stat_0.pmuCmdValue != BMM350_PMU_CMD_STATUS_0_BR)) {
		throw Exception(EX_STATE);
	}

	/* Set FGR to PMU_CMD register */
	pmu_cmd = BMM350_FLUXGUIDE_9MS;

	SMBus::write(BMM350_REG_PMU_CMD, pmu_cmd);
	Timer::sleep(BMM350_FGR_DELAY / 1000);

	/* Verify if PMU_CMD_STATUS_0 register has FGR set */
	getPMUCommandStatus0(pmu_cmd_stat_0);

	if ((pmu_cmd_stat_0.pmuCmdValue != BMM350_PMU_CMD_STATUS_0_FGR)) {
		throw Exception(EX_STATE);
	}

	if ((restore_normal == BMM350_ENABLE)) {
		setPowerMode(BMM350_MODE_NORMAL);
	}
}

void BMM350::readCompensatedData(BMM350Data &data) {
	uint8_t indx;
	float out_data[4] { };
	float dut_offset_coef[3], dut_sensit_coef[3], dut_tco[3], dut_tcs[3];
	float cr_ax_comp_x, cr_ax_comp_y, cr_ax_comp_z;

	/* Reads raw magnetic x,y and z axis along with temperature */
	readOutRawData(out_data);

	/* Apply compensation to temperature reading */
	out_data[3] = (1 + dev.compensate.dut_sensit_coef.t_sens) * out_data[3]
			+ dev.compensate.dut_offset_coef.t_offs;

	/* Store magnetic compensation structure to an array */
	dut_offset_coef[0] = dev.compensate.dut_offset_coef.offset_x;
	dut_offset_coef[1] = dev.compensate.dut_offset_coef.offset_y;
	dut_offset_coef[2] = dev.compensate.dut_offset_coef.offset_z;

	dut_sensit_coef[0] = dev.compensate.dut_sensit_coef.sens_x;
	dut_sensit_coef[1] = dev.compensate.dut_sensit_coef.sens_y;
	dut_sensit_coef[2] = dev.compensate.dut_sensit_coef.sens_z;

	dut_tco[0] = dev.compensate.dut_tco.tco_x;
	dut_tco[1] = dev.compensate.dut_tco.tco_y;
	dut_tco[2] = dev.compensate.dut_tco.tco_z;

	dut_tcs[0] = dev.compensate.dut_tcs.tcs_x;
	dut_tcs[1] = dev.compensate.dut_tcs.tcs_y;
	dut_tcs[2] = dev.compensate.dut_tcs.tcs_z;

	/* Compensate raw magnetic data */
	for (indx = 0; indx < 3; indx++) {
		out_data[indx] *= 1 + dut_sensit_coef[indx];
		out_data[indx] += dut_offset_coef[indx];
		out_data[indx] += dut_tco[indx] * (out_data[3] - dev.compensate.dut_t0);
		out_data[indx] /= 1
				+ dut_tcs[indx] * (out_data[3] - dev.compensate.dut_t0);
	}

	cr_ax_comp_x = (out_data[0]
			- dev.compensate.cross_axis.cross_x_y * out_data[1])
			/ (1
					- dev.compensate.cross_axis.cross_y_x
							* dev.compensate.cross_axis.cross_x_y);
	cr_ax_comp_y = (out_data[1]
			- dev.compensate.cross_axis.cross_y_x * out_data[0])
			/ (1
					- dev.compensate.cross_axis.cross_y_x
							* dev.compensate.cross_axis.cross_x_y);
	cr_ax_comp_z =
			(out_data[2]
					+ (out_data[0]
							* (dev.compensate.cross_axis.cross_y_x
									* dev.compensate.cross_axis.cross_z_y
									- dev.compensate.cross_axis.cross_z_x)
							- out_data[1]
									* (dev.compensate.cross_axis.cross_z_y
											- dev.compensate.cross_axis.cross_x_y
													* dev.compensate.cross_axis.cross_z_x))
							/ (1
									- dev.compensate.cross_axis.cross_y_x
											* dev.compensate.cross_axis.cross_x_y));

	out_data[0] = cr_ax_comp_x;
	out_data[1] = cr_ax_comp_y;
	out_data[2] = cr_ax_comp_z;

	if ((dev.axes & BMM350_EN_X_MSK) == BMM350_DISABLE) {
		data.x = BMM350_DISABLE;
	} else {
		data.x = out_data[0];
	}

	if ((dev.axes & BMM350_EN_Y_MSK) == BMM350_DISABLE) {
		data.y = BMM350_DISABLE;
	} else {
		data.y = out_data[1];
	}

	if ((dev.axes & BMM350_EN_Z_MSK) == BMM350_DISABLE) {
		data.z = BMM350_DISABLE;
	} else {
		data.z = out_data[2];
	}

	data.temperature = out_data[3];
}

void BMM350::setI2CWatchdogTimer(bool enable, bool longDelay) {
	uint8_t reg_data;

	/* Get I2C WDT configuration */
	reg_data = readRegByte(BMM350_REG_I2C_WDT_SET);

	reg_data = BMM350_SET_BITS_POS_0(reg_data, BMM350_I2C_WDT_EN,
			(uint8_t )enable);
	reg_data = BMM350_SET_BITS(reg_data, BMM350_I2C_WDT_SEL,
			(uint8_t )longDelay);

	/* Set I2C WDT configuration */
	SMBus::write(BMM350_REG_I2C_WDT_SET, reg_data);
}

void BMM350::setCtrlUserRegister(bool enable) {
	uint8_t reg_data;

	/* Get control user configuration */
	reg_data = readRegByte(BMM350_REG_CTRL_USER);

	reg_data = BMM350_SET_BITS_POS_0(reg_data, BMM350_CFG_SENS_TIM_AON,
			(uint8_t )enable);

	/* Set control user configuration */
	SMBus::write(BMM350_REG_CTRL_USER, reg_data);
}

void BMM350::getPMUCommandStatus0(BMM350PmuCmdStatus0 &status) {
	uint8_t reg_data;

	/* Get PMU command status 0 data */
	reg_data = readRegByte(BMM350_REG_PMU_CMD_STATUS_0);
	status.pmuCmdBusy = BMM350_GET_BITS_POS_0(reg_data, BMM350_PMU_CMD_BUSY);

	status.odrModified = BMM350_GET_BITS(reg_data, BMM350_ODR_OVWR);

	status.avrModified = BMM350_GET_BITS(reg_data, BMM350_AVG_OVWR);

	status.normalPowerMode = BMM350_GET_BITS(reg_data,
			BMM350_PWR_MODE_IS_NORMAL);

	status.illegalCmd = BMM350_GET_BITS(reg_data, BMM350_CMD_IS_ILLEGAL);

	status.pmuCmdValue = BMM350_GET_BITS(reg_data, BMM350_PMU_CMD_VALUE);
}

unsigned char BMM350::readRegByte(unsigned char command) {
	unsigned char buffer[3];
	auto nRead = SMBus::read(command, 3, buffer);
	if (nRead != 3) {
		throw Exception(EX_OPERATION);
	}

	return buffer[2];
}

unsigned int BMM350::readRegBytes(unsigned char command, unsigned int count,
		void *buffer) {
	if (!buffer || !count || count > 30) {
		throw Exception(EX_ARGUMENT);
	}

	unsigned char in[32];
	auto nRead = SMBus::read(command, (count + 2), in);

	if (nRead != (count + 2)) {
		throw Exception(EX_OPERATION);
	}

	::memcpy(buffer, (in + 2), count);
	return count;
}

void BMM350::readOTPData() {
	uint8_t indx;

	for (indx = 0; indx < OTP_DATA_LENGTH; indx++) {
		dev.otp[indx] = readOTPWord(indx);
	}

	dev.variant = (dev.otp[30] & 0x7f00) >> 9;

	/* Set the default auto bit reset configuration */
	dev.autoBR = (dev.variant <= BMM350_CURRENT_SHUTTLE_VARIANT_ID);

	/* Update magnetometer offset and sensitivity data. */
	updateOffsetAndSensitivity();
}

unsigned short BMM350::readOTPWord(unsigned char addr) {
	uint8_t otp_cmd, otp_status = 0, otp_err = BMM350_OTP_STATUS_NO_ERROR, lsb =
			0, msb = 0;

	/* Set OTP command at specified address */
	otp_cmd = BMM350_OTP_CMD_DIR_READ | (addr & BMM350_OTP_WORD_ADDR_MSK);
	SMBus::write(BMM350_REG_OTP_CMD_REG, otp_cmd);

	do {
		Timer::sleep(0, 300000);

		/* Get OTP status */
		otp_status = readRegByte(BMM350_REG_OTP_STATUS_REG);

		otp_err = BMM350_OTP_STATUS_ERROR(otp_status);
		if (otp_err != BMM350_OTP_STATUS_NO_ERROR) {
			throw Exception(EX_STATE);
		}
	} while ((!(otp_status & BMM350_OTP_STATUS_CMD_DONE)));

	/* Get OTP MSB data */
	msb = readRegByte(BMM350_REG_OTP_DATA_MSB_REG);
	/* Get OTP LSB data */
	lsb = readRegByte(BMM350_REG_OTP_DATA_LSB_REG);
	return ((uint16_t) (msb << 8) | lsb) & 0xFFFF;
}

void BMM350::updateOffsetAndSensitivity() {
	uint16_t off_x_lsb_msb, off_y_lsb_msb, off_z_lsb_msb, t_off = 0;
	uint8_t sens_x, sens_y, sens_z, t_sens = 0;
	uint8_t tco_x, tco_y, tco_z = 0;
	uint8_t tcs_x, tcs_y, tcs_z = 0;
	uint8_t cross_x_y, cross_y_x, cross_z_x, cross_z_y = 0;

	off_x_lsb_msb = dev.otp[BMM350_MAG_OFFSET_X] & 0x0FFF;
	off_y_lsb_msb = ((dev.otp[BMM350_MAG_OFFSET_X] & 0xF000) >> 4)
			+ (dev.otp[BMM350_MAG_OFFSET_Y] & BMM350_LSB_MASK);
	off_z_lsb_msb = (dev.otp[BMM350_MAG_OFFSET_Y] & 0x0F00)
			+ (dev.otp[BMM350_MAG_OFFSET_Z] & BMM350_LSB_MASK);
	t_off = dev.otp[BMM350_TEMP_OFF_SENS] & BMM350_LSB_MASK;

	dev.compensate.dut_offset_coef.offset_x = fix_sign(off_x_lsb_msb,
	BMM350_SIGNED_12_BIT);
	dev.compensate.dut_offset_coef.offset_y = fix_sign(off_y_lsb_msb,
	BMM350_SIGNED_12_BIT);
	dev.compensate.dut_offset_coef.offset_z = fix_sign(off_z_lsb_msb,
	BMM350_SIGNED_12_BIT);
	dev.compensate.dut_offset_coef.t_offs = fix_sign(t_off, BMM350_SIGNED_8_BIT)
			/ 5.0f;

	sens_x = (dev.otp[BMM350_MAG_SENS_X] & BMM350_MSB_MASK) >> 8;
	sens_y = (dev.otp[BMM350_MAG_SENS_Y] & BMM350_LSB_MASK);
	sens_z = (dev.otp[BMM350_MAG_SENS_Z] & BMM350_MSB_MASK) >> 8;
	t_sens = (dev.otp[BMM350_TEMP_OFF_SENS] & BMM350_MSB_MASK) >> 8;

	dev.compensate.dut_sensit_coef.sens_x = fix_sign(sens_x,
	BMM350_SIGNED_8_BIT) / 256.0f;
	dev.compensate.dut_sensit_coef.sens_y = fix_sign(sens_y,
	BMM350_SIGNED_8_BIT) / 256.0f;
	dev.compensate.dut_sensit_coef.sens_z = fix_sign(sens_z,
	BMM350_SIGNED_8_BIT) / 256.0f;
	dev.compensate.dut_sensit_coef.t_sens = fix_sign(t_sens,
	BMM350_SIGNED_8_BIT) / 512.0f;

	tco_x = (dev.otp[BMM350_MAG_TCO_X] & BMM350_LSB_MASK);
	tco_y = (dev.otp[BMM350_MAG_TCO_Y] & BMM350_LSB_MASK);
	tco_z = (dev.otp[BMM350_MAG_TCO_Z] & BMM350_LSB_MASK);

	dev.compensate.dut_tco.tco_x = fix_sign(tco_x, BMM350_SIGNED_8_BIT) / 32.0f;
	dev.compensate.dut_tco.tco_y = fix_sign(tco_y, BMM350_SIGNED_8_BIT) / 32.0f;
	dev.compensate.dut_tco.tco_z = fix_sign(tco_z, BMM350_SIGNED_8_BIT) / 32.0f;

	tcs_x = (dev.otp[BMM350_MAG_TCS_X] & BMM350_MSB_MASK) >> 8;
	tcs_y = (dev.otp[BMM350_MAG_TCS_Y] & BMM350_MSB_MASK) >> 8;
	tcs_z = (dev.otp[BMM350_MAG_TCS_Z] & BMM350_MSB_MASK) >> 8;

	dev.compensate.dut_tcs.tcs_x = fix_sign(tcs_x, BMM350_SIGNED_8_BIT)
			/ 16384.0f;
	dev.compensate.dut_tcs.tcs_y = fix_sign(tcs_y, BMM350_SIGNED_8_BIT)
			/ 16384.0f;
	dev.compensate.dut_tcs.tcs_z = fix_sign(tcs_z, BMM350_SIGNED_8_BIT)
			/ 16384.0f;

	dev.compensate.dut_t0 = (fix_sign(dev.otp[BMM350_MAG_DUT_T_0],
	BMM350_SIGNED_16_BIT) / 512.0f) + 23.0f;

	cross_x_y = (dev.otp[BMM350_CROSS_X_Y] & BMM350_LSB_MASK);
	cross_y_x = (dev.otp[BMM350_CROSS_Y_X] & BMM350_MSB_MASK) >> 8;
	cross_z_x = (dev.otp[BMM350_CROSS_Z_X] & BMM350_LSB_MASK);
	cross_z_y = (dev.otp[BMM350_CROSS_Z_Y] & BMM350_MSB_MASK) >> 8;

	dev.compensate.cross_axis.cross_x_y = fix_sign(cross_x_y,
	BMM350_SIGNED_8_BIT) / 800.0f;
	dev.compensate.cross_axis.cross_y_x = fix_sign(cross_y_x,
	BMM350_SIGNED_8_BIT) / 800.0f;
	dev.compensate.cross_axis.cross_z_x = fix_sign(cross_z_x,
	BMM350_SIGNED_8_BIT) / 800.0f;
	dev.compensate.cross_axis.cross_z_y = fix_sign(cross_z_y,
	BMM350_SIGNED_8_BIT) / 800.0f;
}

void BMM350::setPowerModeInternal(BMM350PowerMode mode) {
	uint8_t reg_data = mode;
	uint8_t get_avg;

	/* Array to store suspend to forced mode delay */
	uint32_t sus_to_forced_mode[4] = { BMM350_SUS_TO_FORCEDMODE_NO_AVG_DELAY,
	BMM350_SUS_TO_FORCEDMODE_AVG_2_DELAY,
	BMM350_SUS_TO_FORCEDMODE_AVG_4_DELAY,
	BMM350_SUS_TO_FORCEDMODE_AVG_8_DELAY };

	/* Array to store suspend to forced mode fast delay */
	uint32_t sus_to_forced_mode_fast[4] = {
	BMM350_SUS_TO_FORCEDMODE_FAST_NO_AVG_DELAY,
	BMM350_SUS_TO_FORCEDMODE_FAST_AVG_2_DELAY,
	BMM350_SUS_TO_FORCEDMODE_FAST_AVG_4_DELAY,
	BMM350_SUS_TO_FORCEDMODE_FAST_AVG_8_DELAY };

	uint8_t avg = 0;
	uint32_t delay_us = 0;

	/* Set PMU command configuration to desired power mode */
	SMBus::write(BMM350_REG_PMU_CMD, reg_data);
	/* Get average configuration */
	get_avg = readRegByte(BMM350_REG_PMU_CMD_AGGR_SET);
	/* Mask the average value */
	avg = ((get_avg & BMM350_AVG_MSK) >> BMM350_AVG_POS);

	/* Check if desired power mode is normal mode */
	if (mode == BMM350_MODE_NORMAL) {
		delay_us = BMM350_SUSPEND_TO_NORMAL_DELAY;
	}

	/* Check if desired power mode is forced mode */
	if (mode == BMM350_MODE_FORCED) {
		/* Store delay based on averaging mode */
		delay_us = sus_to_forced_mode[avg];
	}

	/* Check if desired power mode is forced mode fast */
	if (mode == BMM350_MODE_FAST) {
		/* Store delay based on averaging mode */
		delay_us = sus_to_forced_mode_fast[avg];
	}

	/* Perform delay based on power mode */
	Timer::sleep(delay_us / 1000);
}

void BMM350::readOutRawData(float (&out_data)[4]) {
	/* Float variable to convert mag lsb to uT and temp lsb to degC */
	float lsb_to_ut_degc[4] { };

	BMM350RawData raw_data { };

	readRawData(raw_data);

	/* Convert mag lsb to uT and temp lsb to degC */
	update_default_coefficients(lsb_to_ut_degc);

	out_data[0] = (float) raw_data.x * lsb_to_ut_degc[0];
	out_data[1] = (float) raw_data.y * lsb_to_ut_degc[1];
	out_data[2] = (float) raw_data.z * lsb_to_ut_degc[2];
	out_data[3] = (float) raw_data.temperature * lsb_to_ut_degc[3];

	out_data[3] = (float) (out_data[3] - (1 * 25.49));
}

} /* namespace wanhive */
