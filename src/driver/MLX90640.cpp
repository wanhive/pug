/*
 * MLX90640.cpp
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
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "MLX90640.h"
#include <wanhive/base/common/Exception.h>
#include <wanhive/base/Timer.h>
#include <cmath>
#include <cstring>

#define BIT_MASK(x) (1UL << (x))
#define REG_MASK(sbit,nbits) ~((~(~0UL << (nbits))) << (sbit))

#define MLX90640_EEPROM_START_ADDRESS 0x2400
#define MLX90640_PIXEL_DATA_START_ADDRESS 0x0400
#define MLX90640_LINE_NUM 24
#define MLX90640_COLUMN_NUM 32
#define MLX90640_LINE_SIZE 32
#define MLX90640_COLUMN_SIZE 24
#define MLX90640_AUX_DATA_START_ADDRESS 0x0700
#define MLX90640_STATUS_REG 0x8000
#define MLX90640_INIT_STATUS_VALUE 0x0030
#define MLX90640_STAT_FRAME_MASK BIT_MASK(0)
#define MLX90640_GET_FRAME(reg_value) (reg_value & MLX90640_STAT_FRAME_MASK)
#define MLX90640_STAT_DATA_READY_MASK BIT_MASK(3)
#define MLX90640_GET_DATA_READY(reg_value) (reg_value & MLX90640_STAT_DATA_READY_MASK)

#define MLX90640_CTRL_REG 0x800D
#define MLX90640_CTRL_TRIG_READY_MASK BIT_MASK(15)
#define MLX90640_CTRL_REFRESH_SHIFT 7
#define MLX90640_CTRL_REFRESH_MASK REG_MASK(MLX90640_CTRL_REFRESH_SHIFT,3)
#define MLX90640_CTRL_RESOLUTION_SHIFT 10
#define MLX90640_CTRL_RESOLUTION_MASK REG_MASK(MLX90640_CTRL_RESOLUTION_SHIFT,2)
#define MLX90640_CTRL_MEAS_MODE_SHIFT 12
#define MLX90640_CTRL_MEAS_MODE_MASK BIT_MASK(12)

#define MLX90640_MS_BYTE_SHIFT 8
#define MLX90640_MS_BYTE_MASK 0xFF00
#define MLX90640_LS_BYTE_MASK 0x00FF
#define MLX90640_MS_BYTE(reg16) ((reg16 & MLX90640_MS_BYTE_MASK) >> MLX90640_MS_BYTE_SHIFT)
#define MLX90640_LS_BYTE(reg16) (reg16 & MLX90640_LS_BYTE_MASK)
#define MLX90640_MSBITS_6_MASK 0xFC00
#define MLX90640_LSBITS_10_MASK 0x03FF
#define MLX90640_NIBBLE1_MASK 0x000F
#define MLX90640_NIBBLE2_MASK 0x00F0
#define MLX90640_NIBBLE3_MASK 0x0F00
#define MLX90640_NIBBLE4_MASK 0xF000
#define MLX90640_NIBBLE1(reg16) ((reg16 & MLX90640_NIBBLE1_MASK))
#define MLX90640_NIBBLE2(reg16) ((reg16 & MLX90640_NIBBLE2_MASK) >> 4)
#define MLX90640_NIBBLE3(reg16) ((reg16 & MLX90640_NIBBLE3_MASK) >> 8)
#define MLX90640_NIBBLE4(reg16) ((reg16 & MLX90640_NIBBLE4_MASK) >> 12)

#define POW2(x) pow(2, (double)x)

#define SCALEALPHA 0.000001

namespace wanhive {

MLX90640::MLX90640(unsigned int bus, unsigned int address) :
		MLX9064x(bus, address) {
	setup();
}

MLX90640::MLX90640(const char *path, unsigned int address) :
		MLX9064x(path, address) {
	setup();
}

MLX90640::~MLX90640() {

}

void MLX90640::setup() {
	uint16_t eeData[EEPROM_DUMP_COUNT] { };
	readEEPROM(eeData);
	params.defect = extractParameters(eeData);
	reset();
}

void MLX90640::reset() {
	uint16_t ctrlReg;

	ctrlReg = readReg(MLX90640_CTRL_REG);

	ctrlReg |= MLX90640_CTRL_TRIG_READY_MASK;
	writeReg(MLX90640_CTRL_REG, ctrlReg);

	MLX9064x::reset();

	ctrlReg = readReg(MLX90640_CTRL_REG);

	if ((ctrlReg & MLX90640_CTRL_TRIG_READY_MASK) != 0) {
		throw Exception(EX_OPERATION);
	}
}

void MLX90640::setResolution(MLX90640Resolution resolution) const {
	uint16_t controlRegister1;
	uint16_t value;

	//value = (resolution & 0x03) << 10;
	value = ((uint16_t) resolution << MLX90640_CTRL_RESOLUTION_SHIFT);
	value &= ~MLX90640_CTRL_RESOLUTION_MASK;

	controlRegister1 = readReg(MLX90640_CTRL_REG);
	value = (controlRegister1 & MLX90640_CTRL_RESOLUTION_MASK) | value;
	writeReg(MLX90640_CTRL_REG, value);
}

MLX90640Resolution MLX90640::getResolution() const {
	uint16_t controlRegister1;
	controlRegister1 = readReg(MLX90640_CTRL_REG);
	auto value = ((controlRegister1 & ~MLX90640_CTRL_RESOLUTION_MASK)
			>> MLX90640_CTRL_RESOLUTION_SHIFT);
	return static_cast<MLX90640Resolution>(value & 0x03);
}

void MLX90640::setRefreshRate(MLX90640RefreshRate refreshRate) const {
	uint16_t controlRegister1;
	uint16_t value;

	value = ((uint16_t) refreshRate << MLX90640_CTRL_REFRESH_SHIFT);
	value &= ~MLX90640_CTRL_REFRESH_MASK;

	controlRegister1 = readReg(MLX90640_CTRL_REG);
	value = (controlRegister1 & MLX90640_CTRL_REFRESH_MASK) | value;
	writeReg(MLX90640_CTRL_REG, value);
}

MLX90640RefreshRate MLX90640::getRefreshRate() const {
	uint16_t controlRegister1;

	controlRegister1 = readReg(MLX90640_CTRL_REG);
	auto value = ((controlRegister1 & ~MLX90640_CTRL_REFRESH_MASK)
			>> MLX90640_CTRL_REFRESH_SHIFT);
	return static_cast<MLX90640RefreshRate>(value & 0x07);
}

unsigned int MLX90640::calculateDelay() const {
	auto rr = getRefreshRate();
	auto hz = (0.5f) * (1UL << rr);
	return static_cast<unsigned int>(ceilf(1000 / hz));
}

void MLX90640::setMode(MLX90640Mode mode) const {
	uint16_t controlRegister1;
	uint16_t value;

	controlRegister1 = readReg(MLX90640_CTRL_REG);
	switch (mode) {
	case MLX90640_INTERLEAVE:
		value = (controlRegister1 & ~MLX90640_CTRL_MEAS_MODE_MASK);
		writeReg(MLX90640_CTRL_REG, value);
		break;
	case MLX90640_CHESS:
		value = (controlRegister1 | MLX90640_CTRL_MEAS_MODE_MASK);
		writeReg(MLX90640_CTRL_REG, value);
		break;
	default:
		break;
	}
}

MLX90640Mode MLX90640::getMode() const {
	uint16_t controlRegister1;

	controlRegister1 = readReg(MLX90640_CTRL_REG);
	auto value = ((controlRegister1 & MLX90640_CTRL_MEAS_MODE_MASK)
			>> MLX90640_CTRL_MEAS_MODE_SHIFT);
	return static_cast<MLX90640Mode>(value & 0x01);
}

void MLX90640::synchronizeFrame() const {
	uint16_t dataReady = 0;
	uint16_t statusRegister;
	writeReg(MLX90640_STATUS_REG, MLX90640_INIT_STATUS_VALUE);

	while (dataReady == 0) {
		statusRegister = readReg(MLX90640_STATUS_REG);
		dataReady = MLX90640_GET_DATA_READY(statusRegister);
	}
}

bool MLX90640::readFrame(MLX90640Frame &frame, bool wait) const {
	if (readFrameData(frame.data, wait)) {
		frame.vdd = getVdd(frame.data);
		frame.ta = getAmbientTemperature(frame.data, frame.vdd);
		return true;
	} else {
		return false;
	}
}

bool MLX90640::getTemperature(MLX90640Data &result, bool wait) const {
	MLX90640Frame frame;
	if (readFrame(frame, wait)) {
		getTemperature(frame, 0.95, (frame.ta - TA_SHIFT), result);
		return true;
	} else {
		return false;
	}
}

bool MLX90640::getTemperature(MLX90640Data &result, float emissivity, float tr,
		bool wait) const {
	MLX90640Frame frame;
	if (readFrame(frame, wait)) {
		getTemperature(frame, emissivity, tr, result);
		return true;
	} else {
		return false;
	}
}

void MLX90640::getTemperature(const MLX90640Frame &frame, float emissivity,
		float tr, MLX90640Data &result) const noexcept {
	float vdd;
	float ta;
	float ta4;
	float tr4;
	float taTr;
	float gain;
	float irDataCP[2];
	float irData;
	float alphaCompensated;
	uint8_t mode;
	int8_t ilPattern;
	int8_t chessPattern;
	int8_t pattern;
	int8_t conversionPattern;
	float Sx;
	float To;
	float alphaCorrR[4];
	int8_t range;
	uint16_t subPage;
	float ktaScale;
	float kvScale;
	float alphaScale;
	float kta;
	float kv;

	auto frameData = frame.data;
	result.max = 0;
	result.min = 0;

	subPage = frameData[833];
	vdd = frame.vdd;
	ta = frame.ta;

	ta4 = (ta + 273.15);
	ta4 = ta4 * ta4;
	ta4 = ta4 * ta4;
	tr4 = (tr + 273.15);
	tr4 = tr4 * tr4;
	tr4 = tr4 * tr4;
	taTr = tr4 - (tr4 - ta4) / emissivity;

	ktaScale = POW2(params.ktaScale);
	kvScale = POW2(params.kvScale);
	alphaScale = POW2(params.alphaScale);

	alphaCorrR[0] = 1 / (1 + params.ksTo[0] * 40);
	alphaCorrR[1] = 1;
	alphaCorrR[2] = (1 + params.ksTo[1] * params.ct[2]);
	alphaCorrR[3] = alphaCorrR[2]
			* (1 + params.ksTo[2] * (params.ct[3] - params.ct[2]));

	//------------------------- Gain calculation -----------------------------------

	gain = (float) params.gainEE / (int16_t) frameData[778];

	//------------------------- To calculation -------------------------------------
	mode = (frameData[832] & MLX90640_CTRL_MEAS_MODE_MASK) >> 5;

	irDataCP[0] = (int16_t) frameData[776] * gain;
	irDataCP[1] = (int16_t) frameData[808] * gain;

	irDataCP[0] = irDataCP[0]
			- params.cpOffset[0] * (1 + params.cpKta * (ta - 25))
					* (1 + params.cpKv * (vdd - 3.3));
	if (mode == params.calibrationModeEE) {
		irDataCP[1] = irDataCP[1]
				- params.cpOffset[1] * (1 + params.cpKta * (ta - 25))
						* (1 + params.cpKv * (vdd - 3.3));
	} else {
		irDataCP[1] = irDataCP[1]
				- (params.cpOffset[1] + params.ilChessC[0])
						* (1 + params.cpKta * (ta - 25))
						* (1 + params.cpKv * (vdd - 3.3));
	}

	for (int pixelNumber = 0; pixelNumber < 768; pixelNumber++) {
		ilPattern = pixelNumber / 32 - (pixelNumber / 64) * 2;
		chessPattern = ilPattern ^ (pixelNumber - (pixelNumber / 2) * 2);
		conversionPattern = ((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4
				+ (pixelNumber + 1) / 4 - pixelNumber / 4)
				* (1 - 2 * ilPattern);

		if (mode == 0) {
			pattern = ilPattern;
		} else {
			pattern = chessPattern;
		}

		if (pattern == frameData[833]) {
			irData = (int16_t) frameData[pixelNumber] * gain;

			kta = params.kta[pixelNumber] / ktaScale;
			kv = params.kv[pixelNumber] / kvScale;
			irData = irData
					- params.offset[pixelNumber] * (1 + kta * (ta - 25))
							* (1 + kv * (vdd - 3.3));

			if (mode != params.calibrationModeEE) {
				irData = irData + params.ilChessC[2] * (2 * ilPattern - 1)
						- params.ilChessC[1] * conversionPattern;
			}

			irData = irData - params.tgc * irDataCP[subPage];
			irData = irData / emissivity;

			alphaCompensated = SCALEALPHA * alphaScale
					/ params.alpha[pixelNumber];
			alphaCompensated = alphaCompensated * (1 + params.KsTa * (ta - 25));

			Sx = alphaCompensated * alphaCompensated * alphaCompensated
					* (irData + alphaCompensated * taTr);
			Sx = sqrt(sqrt(Sx)) * params.ksTo[1];

			To =
					sqrt(
							sqrt(
									irData
											/ (alphaCompensated
													* (1
															- params.ksTo[1]
																	* 273.15)
													+ Sx) + taTr)) - 273.15;

			if (To < params.ct[1]) {
				range = 0;
			} else if (To < params.ct[2]) {
				range = 1;
			} else if (To < params.ct[3]) {
				range = 2;
			} else {
				range = 3;
			}

			To =
					sqrt(
							sqrt(
									irData
											/ (alphaCompensated
													* alphaCorrR[range]
													* (1
															+ params.ksTo[range]
																	* (To
																			- params.ct[range])))
											+ taTr)) - 273.15;

			result.data[pixelNumber] = To;
			if (To > result.data[result.max]) {
				result.max = pixelNumber;
			} else {
				result.min = pixelNumber;
			}
		}
	}
}

bool MLX90640::getImage(MLX90640Data &result, bool wait) const {
	MLX90640Frame frame;
	if (readFrame(frame, wait)) {
		getImage(frame, result);
		return true;
	} else {
		return false;
	}
}

void MLX90640::getImage(const MLX90640Frame &frame,
		MLX90640Data &result) const noexcept {
	float vdd;
	float ta;
	float gain;
	float irDataCP[2];
	float irData;
	float alphaCompensated;
	uint8_t mode;
	int8_t ilPattern;
	int8_t chessPattern;
	int8_t pattern;
	int8_t conversionPattern;
	float image;
	uint16_t subPage;
	float ktaScale;
	float kvScale;
	float kta;
	float kv;

	auto frameData = frame.data;
	subPage = frameData[833];
	result.max = 0;
	result.min = 0;
	vdd = frame.vdd;
	ta = frame.ta;

	ktaScale = POW2(params.ktaScale);
	kvScale = POW2(params.kvScale);

	//------------------------- Gain calculation -----------------------------------

	gain = (float) params.gainEE / (int16_t) frameData[778];

	//------------------------- Image calculation -------------------------------------

	mode = (frameData[832] & MLX90640_CTRL_MEAS_MODE_MASK) >> 5;

	irDataCP[0] = (int16_t) frameData[776] * gain;
	irDataCP[1] = (int16_t) frameData[808] * gain;

	irDataCP[0] = irDataCP[0]
			- params.cpOffset[0] * (1 + params.cpKta * (ta - 25))
					* (1 + params.cpKv * (vdd - 3.3));
	if (mode == params.calibrationModeEE) {
		irDataCP[1] = irDataCP[1]
				- params.cpOffset[1] * (1 + params.cpKta * (ta - 25))
						* (1 + params.cpKv * (vdd - 3.3));
	} else {
		irDataCP[1] = irDataCP[1]
				- (params.cpOffset[1] + params.ilChessC[0])
						* (1 + params.cpKta * (ta - 25))
						* (1 + params.cpKv * (vdd - 3.3));
	}

	for (int pixelNumber = 0; pixelNumber < 768; pixelNumber++) {
		ilPattern = pixelNumber / 32 - (pixelNumber / 64) * 2;
		chessPattern = ilPattern ^ (pixelNumber - (pixelNumber / 2) * 2);
		conversionPattern = ((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4
				+ (pixelNumber + 1) / 4 - pixelNumber / 4)
				* (1 - 2 * ilPattern);

		if (mode == 0) {
			pattern = ilPattern;
		} else {
			pattern = chessPattern;
		}

		if (pattern == frameData[833]) {
			irData = (int16_t) frameData[pixelNumber] * gain;

			kta = params.kta[pixelNumber] / ktaScale;
			kv = params.kv[pixelNumber] / kvScale;
			irData = irData
					- params.offset[pixelNumber] * (1 + kta * (ta - 25))
							* (1 + kv * (vdd - 3.3));

			if (mode != params.calibrationModeEE) {
				irData = irData + params.ilChessC[2] * (2 * ilPattern - 1)
						- params.ilChessC[1] * conversionPattern;
			}

			irData = irData - params.tgc * irDataCP[subPage];

			alphaCompensated = params.alpha[pixelNumber];

			image = irData * alphaCompensated;

			result.data[pixelNumber] = image;
			if (image > result.data[result.max]) {
				result.max = pixelNumber;
			} else {
				result.min = pixelNumber;
			}
		}
	}
}

void MLX90640::fixBrokenPixels(MLX90640Mode mode,
		MLX90640Data &target) const noexcept {
	badPixelsCorrection(params.brokenPixels, mode, target);
}

void MLX90640::fixOutlierPixels(MLX90640Mode mode,
		MLX90640Data &target) const noexcept {
	badPixelsCorrection(params.outlierPixels, mode, target);
}

MLX90640Defect MLX90640::getDefect() const noexcept {
	return params.defect;
}

unsigned int MLX90640::getSubPageNumber(const MLX90640Frame &frame) noexcept {
	return (frame.data[833] & 0x01);
}

MLX90640Mode MLX90640::getMode(const MLX90640Frame &frame) noexcept {
	auto value = ((frame.data[832] & MLX90640_CTRL_MEAS_MODE_MASK)
			>> MLX90640_CTRL_MEAS_MODE_SHIFT);
	return static_cast<MLX90640Mode>(value & 0x01);
}

void MLX90640::readEEPROM(uint16_t *eeData) {
	readReg(MLX90640_EEPROM_START_ADDRESS, EEPROM_DUMP_COUNT, eeData);
}

MLX90640Defect MLX90640::extractParameters(const uint16_t *eeData) noexcept {
	extractVDDParameters(eeData);
	extractPTATParameters(eeData);
	extractGainParameters(eeData);
	extractTgcParameters(eeData);
	extractResolutionParameters(eeData);
	extractKsTaParameters(eeData);
	extractKsToParameters(eeData);
	extractCPParameters(eeData);
	extractAlphaParameters(eeData);
	extractOffsetParameters(eeData);
	extractKtaPixelParameters(eeData);
	extractKvPixelParameters(eeData);
	extractCILCParameters(eeData);
	return extractDeviatingPixels(eeData);
}

bool MLX90640::readFrameData(uint16_t *frameData, bool wait) const {
	uint16_t controlRegister1;
	uint16_t statusRegister;
	uint16_t aux[AUX_DATA_COUNT];
	uint8_t cnt = 0;

	while (true) {
		statusRegister = readReg(MLX90640_STATUS_REG);
		auto dataReady = MLX90640_GET_DATA_READY(statusRegister);
		if (dataReady) {
			break;
		} else if (wait) {
			continue;
		} else {
			return false;
		}
	}

	writeReg(MLX90640_STATUS_REG, MLX90640_INIT_STATUS_VALUE);
	readReg(MLX90640_PIXEL_DATA_START_ADDRESS, PIXELS, frameData);
	readReg(MLX90640_AUX_DATA_START_ADDRESS, AUX_DATA_COUNT, aux);

	controlRegister1 = readReg(MLX90640_CTRL_REG);
	frameData[832] = controlRegister1;
	frameData[833] = MLX90640_GET_FRAME(statusRegister);

	if (validateAuxData(aux)) {
		for (cnt = 0; cnt < AUX_DATA_COUNT; cnt++) {
			frameData[cnt + PIXELS] = aux[cnt];
		}
	}

	if (!validateFrameData(frameData)) {
		throw Exception(EX_OPERATION);
	}

	return true;
}

float MLX90640::getVdd(const uint16_t *frameData) const noexcept {
	float vdd;
	float resolutionCorrection;

	uint16_t resolutionRAM = (frameData[832] & ~MLX90640_CTRL_RESOLUTION_MASK)
			>> MLX90640_CTRL_RESOLUTION_SHIFT;
	resolutionCorrection = POW2(params.resolutionEE) / POW2(resolutionRAM);
	vdd = (resolutionCorrection * (int16_t) frameData[810] - params.vdd25)
			/ params.kVdd + 3.3;

	return vdd;
}

float MLX90640::getAmbientTemperature(const uint16_t *frameData,
		float vdd) const noexcept {
	int16_t ptat;
	float ptatArt;
	float ta;

	ptat = (int16_t) frameData[800];

	ptatArt = (ptat / (ptat * params.alphaPTAT + (int16_t) frameData[768]))
			* POW2(18);

	ta = (ptatArt / (1 + params.KvPTAT * (vdd - 3.3)) - params.vPTAT25);
	ta = ta / params.KtPTAT + 25;

	return ta;
}

void MLX90640::extractVDDParameters(const uint16_t *eeData) noexcept {
	int8_t kVdd;
	int16_t vdd25;

	kVdd = MLX90640_MS_BYTE(eeData[51]);

	vdd25 = MLX90640_LS_BYTE(eeData[51]);
	vdd25 = ((vdd25 - 256) << 5) - 8192;

	params.kVdd = 32 * kVdd;
	params.vdd25 = vdd25;
}

void MLX90640::extractPTATParameters(const uint16_t *eeData) noexcept {
	float KvPTAT;
	float KtPTAT;
	int16_t vPTAT25;
	float alphaPTAT;

	KvPTAT = (eeData[50] & MLX90640_MSBITS_6_MASK) >> 10;
	if (KvPTAT > 31) {
		KvPTAT = KvPTAT - 64;
	}
	KvPTAT = KvPTAT / 4096;

	KtPTAT = eeData[50] & MLX90640_LSBITS_10_MASK;
	if (KtPTAT > 511) {
		KtPTAT = KtPTAT - 1024;
	}
	KtPTAT = KtPTAT / 8;

	vPTAT25 = eeData[49];

	alphaPTAT = (eeData[16] & MLX90640_NIBBLE4_MASK) / POW2(14) + 8.0f;

	params.KvPTAT = KvPTAT;
	params.KtPTAT = KtPTAT;
	params.vPTAT25 = vPTAT25;
	params.alphaPTAT = alphaPTAT;
}

void MLX90640::extractGainParameters(const uint16_t *eeData) noexcept {
	params.gainEE = (int16_t) eeData[48];
}

void MLX90640::extractTgcParameters(const uint16_t *eeData) noexcept {
	params.tgc = (int8_t) MLX90640_LS_BYTE(eeData[60]) / 32.0f;
}

void MLX90640::extractResolutionParameters(const uint16_t *eeData) noexcept {
	uint8_t resolutionEE;
	resolutionEE = (eeData[56] & 0x3000) >> 12;

	params.resolutionEE = resolutionEE;
}

void MLX90640::extractKsTaParameters(const uint16_t *eeData) noexcept {
	params.KsTa = (int8_t) MLX90640_MS_BYTE(eeData[60]) / 8192.0f;
}

void MLX90640::extractKsToParameters(const uint16_t *eeData) noexcept {
	int32_t KsToScale;
	int8_t step;

	step = ((eeData[63] & 0x3000) >> 12) * 10;

	params.ct[0] = -40;
	params.ct[1] = 0;
	params.ct[2] = MLX90640_NIBBLE2(eeData[63]);
	params.ct[3] = MLX90640_NIBBLE3(eeData[63]);

	params.ct[2] = params.ct[2] * step;
	params.ct[3] = params.ct[2] + params.ct[3] * step;
	params.ct[4] = 400;

	KsToScale = MLX90640_NIBBLE1(eeData[63]) + 8;
	KsToScale = 1UL << KsToScale;

	params.ksTo[0] = (int8_t) MLX90640_LS_BYTE(eeData[61]) / (float) KsToScale;
	params.ksTo[1] = (int8_t) MLX90640_MS_BYTE(eeData[61]) / (float) KsToScale;
	params.ksTo[2] = (int8_t) MLX90640_LS_BYTE(eeData[62]) / (float) KsToScale;
	params.ksTo[3] = (int8_t) MLX90640_MS_BYTE(eeData[62]) / (float) KsToScale;
	params.ksTo[4] = -0.0002;
}

void MLX90640::extractAlphaParameters(const uint16_t *eeData) noexcept {
	int accRow[24];
	int accColumn[32];
	int p = 0;
	int alphaRef;
	uint8_t alphaScale;
	uint8_t accRowScale;
	uint8_t accColumnScale;
	uint8_t accRemScale;
	float alphaTemp[768];
	float temp;

	accRemScale = MLX90640_NIBBLE1(eeData[32]);
	accColumnScale = MLX90640_NIBBLE2(eeData[32]);
	accRowScale = MLX90640_NIBBLE3(eeData[32]);
	alphaScale = MLX90640_NIBBLE4(eeData[32]) + 30;
	alphaRef = eeData[33];

	for (int i = 0; i < 6; i++) {
		p = i * 4;
		accRow[p + 0] = MLX90640_NIBBLE1(eeData[34 + i]);
		accRow[p + 1] = MLX90640_NIBBLE2(eeData[34 + i]);
		accRow[p + 2] = MLX90640_NIBBLE3(eeData[34 + i]);
		accRow[p + 3] = MLX90640_NIBBLE4(eeData[34 + i]);
	}

	for (int i = 0; i < MLX90640_LINE_NUM; i++) {
		if (accRow[i] > 7) {
			accRow[i] = accRow[i] - 16;
		}
	}

	for (int i = 0; i < 8; i++) {
		p = i * 4;
		accColumn[p + 0] = MLX90640_NIBBLE1(eeData[40 + i]);
		accColumn[p + 1] = MLX90640_NIBBLE2(eeData[40 + i]);
		accColumn[p + 2] = MLX90640_NIBBLE3(eeData[40 + i]);
		accColumn[p + 3] = MLX90640_NIBBLE4(eeData[40 + i]);
	}

	for (int i = 0; i < MLX90640_COLUMN_NUM; i++) {
		if (accColumn[i] > 7) {
			accColumn[i] = accColumn[i] - 16;
		}
	}

	for (int i = 0; i < MLX90640_LINE_NUM; i++) {
		for (int j = 0; j < MLX90640_COLUMN_NUM; j++) {
			p = 32 * i + j;
			alphaTemp[p] = (eeData[64 + p] & 0x03F0) >> 4;
			if (alphaTemp[p] > 31) {
				alphaTemp[p] = alphaTemp[p] - 64;
			}
			alphaTemp[p] = alphaTemp[p] * (1 << accRemScale);
			alphaTemp[p] = (alphaRef + (accRow[i] << accRowScale)
					+ (accColumn[j] << accColumnScale) + alphaTemp[p]);
			alphaTemp[p] = alphaTemp[p] / POW2(alphaScale);
			alphaTemp[p] = alphaTemp[p]
					- params.tgc * (params.cpAlpha[0] + params.cpAlpha[1]) / 2;
			alphaTemp[p] = SCALEALPHA / alphaTemp[p];
		}
	}

	temp = alphaTemp[0];
	for (unsigned i = 1; i < PIXELS; i++) {
		if (alphaTemp[i] > temp) {
			temp = alphaTemp[i];
		}
	}

	alphaScale = 0;
	while (temp < 32767.4) {
		temp = temp * 2;
		alphaScale = alphaScale + 1;
	}

	for (unsigned i = 0; i < PIXELS; i++) {
		temp = alphaTemp[i] * POW2(alphaScale);
		params.alpha[i] = (temp + 0.5);

	}

	params.alphaScale = alphaScale;
}

void MLX90640::extractOffsetParameters(const uint16_t *eeData) noexcept {
	int occRow[24];
	int occColumn[32];
	int p = 0;
	int16_t offsetRef;
	uint8_t occRowScale;
	uint8_t occColumnScale;
	uint8_t occRemScale;

	occRemScale = MLX90640_NIBBLE1(eeData[16]);
	occColumnScale = MLX90640_NIBBLE2(eeData[16]);
	occRowScale = MLX90640_NIBBLE3(eeData[16]);
	offsetRef = (int16_t) eeData[17];

	for (int i = 0; i < 6; i++) {
		p = i * 4;
		occRow[p + 0] = MLX90640_NIBBLE1(eeData[18 + i]);
		occRow[p + 1] = MLX90640_NIBBLE2(eeData[18 + i]);
		occRow[p + 2] = MLX90640_NIBBLE3(eeData[18 + i]);
		occRow[p + 3] = MLX90640_NIBBLE4(eeData[18 + i]);
	}

	for (int i = 0; i < MLX90640_LINE_NUM; i++) {
		if (occRow[i] > 7) {
			occRow[i] = occRow[i] - 16;
		}
	}

	for (int i = 0; i < 8; i++) {
		p = i * 4;
		occColumn[p + 0] = MLX90640_NIBBLE1(eeData[24 + i]);
		occColumn[p + 1] = MLX90640_NIBBLE2(eeData[24 + i]);
		occColumn[p + 2] = MLX90640_NIBBLE3(eeData[24 + i]);
		occColumn[p + 3] = MLX90640_NIBBLE4(eeData[24 + i]);
	}

	for (int i = 0; i < MLX90640_COLUMN_NUM; i++) {
		if (occColumn[i] > 7) {
			occColumn[i] = occColumn[i] - 16;
		}
	}

	for (int i = 0; i < MLX90640_LINE_NUM; i++) {
		for (int j = 0; j < MLX90640_COLUMN_NUM; j++) {
			p = 32 * i + j;
			params.offset[p] = (eeData[64 + p] & MLX90640_MSBITS_6_MASK) >> 10;
			if (params.offset[p] > 31) {
				params.offset[p] = params.offset[p] - 64;
			}
			params.offset[p] = params.offset[p] * (1 << occRemScale);
			params.offset[p] = (offsetRef + (occRow[i] << occRowScale)
					+ (occColumn[j] << occColumnScale) + params.offset[p]);
		}
	}
}

void MLX90640::extractKtaPixelParameters(const uint16_t *eeData) noexcept {
	int p = 0;
	int8_t KtaRC[4];
	uint8_t ktaScale1;
	uint8_t ktaScale2;
	uint8_t split;
	float ktaTemp[768];
	float temp;

	KtaRC[0] = (int8_t) MLX90640_MS_BYTE(eeData[54]);
	;
	KtaRC[2] = (int8_t) MLX90640_LS_BYTE(eeData[54]);
	;
	KtaRC[1] = (int8_t) MLX90640_MS_BYTE(eeData[55]);
	;
	KtaRC[3] = (int8_t) MLX90640_LS_BYTE(eeData[55]);
	;

	ktaScale1 = MLX90640_NIBBLE2(eeData[56]) + 8;
	ktaScale2 = MLX90640_NIBBLE1(eeData[56]);

	for (int i = 0; i < MLX90640_LINE_NUM; i++) {
		for (int j = 0; j < MLX90640_COLUMN_NUM; j++) {
			p = 32 * i + j;
			split = 2 * (p / 32 - (p / 64) * 2) + p % 2;
			ktaTemp[p] = (eeData[64 + p] & 0x000E) >> 1;
			if (ktaTemp[p] > 3) {
				ktaTemp[p] = ktaTemp[p] - 8;
			}
			ktaTemp[p] = ktaTemp[p] * (1 << ktaScale2);
			ktaTemp[p] = KtaRC[split] + ktaTemp[p];
			ktaTemp[p] = ktaTemp[p] / POW2(ktaScale1);

		}
	}

	temp = fabs(ktaTemp[0]);
	for (unsigned i = 1; i < PIXELS; i++) {
		if (fabs(ktaTemp[i]) > temp) {
			temp = fabs(ktaTemp[i]);
		}
	}

	ktaScale1 = 0;
	while (temp < 63.4) {
		temp = temp * 2;
		ktaScale1 = ktaScale1 + 1;
	}

	for (unsigned i = 0; i < PIXELS; i++) {
		temp = ktaTemp[i] * POW2(ktaScale1);
		if (temp < 0) {
			params.kta[i] = (temp - 0.5);
		} else {
			params.kta[i] = (temp + 0.5);
		}

	}

	params.ktaScale = ktaScale1;
}

void MLX90640::extractKvPixelParameters(const uint16_t *eeData) noexcept {
	int p = 0;
	int8_t KvT[4];
	int8_t KvRoCo;
	int8_t KvRoCe;
	int8_t KvReCo;
	int8_t KvReCe;
	uint8_t kvScale;
	uint8_t split;
	float kvTemp[768];
	float temp;

	KvRoCo = MLX90640_NIBBLE4(eeData[52]);
	if (KvRoCo > 7) {
		KvRoCo = KvRoCo - 16;
	}
	KvT[0] = KvRoCo;

	KvReCo = MLX90640_NIBBLE3(eeData[52]);
	if (KvReCo > 7) {
		KvReCo = KvReCo - 16;
	}
	KvT[2] = KvReCo;

	KvRoCe = MLX90640_NIBBLE2(eeData[52]);
	if (KvRoCe > 7) {
		KvRoCe = KvRoCe - 16;
	}
	KvT[1] = KvRoCe;

	KvReCe = MLX90640_NIBBLE1(eeData[52]);
	if (KvReCe > 7) {
		KvReCe = KvReCe - 16;
	}
	KvT[3] = KvReCe;

	kvScale = MLX90640_NIBBLE3(eeData[56]);

	for (int i = 0; i < MLX90640_LINE_NUM; i++) {
		for (int j = 0; j < MLX90640_COLUMN_NUM; j++) {
			p = 32 * i + j;
			split = 2 * (p / 32 - (p / 64) * 2) + p % 2;
			kvTemp[p] = KvT[split];
			kvTemp[p] = kvTemp[p] / POW2(kvScale);
		}
	}

	temp = fabs(kvTemp[0]);
	for (unsigned i = 1; i < PIXELS; i++) {
		if (fabs(kvTemp[i]) > temp) {
			temp = fabs(kvTemp[i]);
		}
	}

	kvScale = 0;
	while (temp < 63.4) {
		temp = temp * 2;
		kvScale = kvScale + 1;
	}

	for (unsigned i = 0; i < PIXELS; i++) {
		temp = kvTemp[i] * POW2(kvScale);
		if (temp < 0) {
			params.kv[i] = (temp - 0.5);
		} else {
			params.kv[i] = (temp + 0.5);
		}

	}

	params.kvScale = kvScale;
}

void MLX90640::extractCPParameters(const uint16_t *eeData) noexcept {
	float alphaSP[2];
	int16_t offsetSP[2];
	float cpKv;
	float cpKta;
	uint8_t alphaScale;
	uint8_t ktaScale1;
	uint8_t kvScale;

	alphaScale = MLX90640_NIBBLE4(eeData[32]) + 27;

	offsetSP[0] = (eeData[58] & MLX90640_LSBITS_10_MASK);
	if (offsetSP[0] > 511) {
		offsetSP[0] = offsetSP[0] - 1024;
	}

	offsetSP[1] = (eeData[58] & MLX90640_MSBITS_6_MASK) >> 10;
	if (offsetSP[1] > 31) {
		offsetSP[1] = offsetSP[1] - 64;
	}
	offsetSP[1] = offsetSP[1] + offsetSP[0];

	alphaSP[0] = (eeData[57] & MLX90640_LSBITS_10_MASK);
	if (alphaSP[0] > 511) {
		alphaSP[0] = alphaSP[0] - 1024;
	}
	alphaSP[0] = alphaSP[0] / POW2(alphaScale);

	alphaSP[1] = (eeData[57] & MLX90640_MSBITS_6_MASK) >> 10;
	if (alphaSP[1] > 31) {
		alphaSP[1] = alphaSP[1] - 64;
	}
	alphaSP[1] = (1 + alphaSP[1] / 128) * alphaSP[0];

	cpKta = (int8_t) MLX90640_LS_BYTE(eeData[59]);

	ktaScale1 = MLX90640_NIBBLE2(eeData[56]) + 8;
	params.cpKta = cpKta / POW2(ktaScale1);

	cpKv = (int8_t) MLX90640_MS_BYTE(eeData[59]);

	kvScale = MLX90640_NIBBLE3(eeData[56]);
	params.cpKv = cpKv / POW2(kvScale);

	params.cpAlpha[0] = alphaSP[0];
	params.cpAlpha[1] = alphaSP[1];
	params.cpOffset[0] = offsetSP[0];
	params.cpOffset[1] = offsetSP[1];
}

void MLX90640::extractCILCParameters(const uint16_t *eeData) noexcept {
	float ilChessC[3];
	uint8_t calibrationModeEE;

	calibrationModeEE = (eeData[10] & 0x0800) >> 4;
	calibrationModeEE = calibrationModeEE ^ 0x80;

	ilChessC[0] = (eeData[53] & 0x003F);
	if (ilChessC[0] > 31) {
		ilChessC[0] = ilChessC[0] - 64;
	}
	ilChessC[0] = ilChessC[0] / 16.0f;

	ilChessC[1] = (eeData[53] & 0x07C0) >> 6;
	if (ilChessC[1] > 15) {
		ilChessC[1] = ilChessC[1] - 32;
	}
	ilChessC[1] = ilChessC[1] / 2.0f;

	ilChessC[2] = (eeData[53] & 0xF800) >> 11;
	if (ilChessC[2] > 15) {
		ilChessC[2] = ilChessC[2] - 32;
	}
	ilChessC[2] = ilChessC[2] / 8.0f;

	params.calibrationModeEE = calibrationModeEE;
	params.ilChessC[0] = ilChessC[0];
	params.ilChessC[1] = ilChessC[1];
	params.ilChessC[2] = ilChessC[2];
}

MLX90640Defect MLX90640::extractDeviatingPixels(const uint16_t *eeData) noexcept {
	uint16_t pixCnt = 0;
	uint16_t brokenPixCnt = 0;
	uint16_t outlierPixCnt = 0;
	MLX90640Defect warn = MLX90640_PIX_OK;
	int i;

	for (pixCnt = 0; pixCnt < 5; pixCnt++) {
		params.brokenPixels[pixCnt] = 0xFFFF;
		params.outlierPixels[pixCnt] = 0xFFFF;
	}

	pixCnt = 0;
	while (pixCnt < PIXELS && brokenPixCnt < 5 && outlierPixCnt < 5) {
		if (eeData[pixCnt + 64] == 0) {
			params.brokenPixels[brokenPixCnt] = pixCnt;
			brokenPixCnt = brokenPixCnt + 1;
		} else if ((eeData[pixCnt + 64] & 0x0001) != 0) {
			params.outlierPixels[outlierPixCnt] = pixCnt;
			outlierPixCnt = outlierPixCnt + 1;
		}

		pixCnt = pixCnt + 1;

	}

	if (brokenPixCnt > 4) {
		warn = MLX90640_PIX_BROKEN;
	} else if (outlierPixCnt > 4) {
		warn = MLX90640_PIX_OUTLIER;
	} else if ((brokenPixCnt + outlierPixCnt) > 4) {
		warn = MLX90640_PIX_BAD;
	} else {
		for (pixCnt = 0; pixCnt < brokenPixCnt; pixCnt++) {
			for (i = pixCnt + 1; i < brokenPixCnt; i++) {
				if (!checkAdjacentPixels(params.brokenPixels[pixCnt],
						params.brokenPixels[i])) {
					return MLX90640_PIX_ADJACENT;
				}
			}
		}

		for (pixCnt = 0; pixCnt < outlierPixCnt; pixCnt++) {
			for (i = pixCnt + 1; i < outlierPixCnt; i++) {
				if (!checkAdjacentPixels(params.outlierPixels[pixCnt],
						params.outlierPixels[i])) {
					return MLX90640_PIX_ADJACENT;
				}
			}
		}

		for (pixCnt = 0; pixCnt < brokenPixCnt; pixCnt++) {
			for (i = 0; i < outlierPixCnt; i++) {
				if (!checkAdjacentPixels(params.brokenPixels[pixCnt],
						params.outlierPixels[i])) {
					return MLX90640_PIX_ADJACENT;
				}
			}
		}

	}

	return warn;
}

bool MLX90640::checkAdjacentPixels(uint16_t pix1, uint16_t pix2) const noexcept {
	int pixPosDif;
	uint16_t lp1 = pix1 >> 5;
	uint16_t lp2 = pix2 >> 5;
	uint16_t cp1 = pix1 - (lp1 << 5);
	uint16_t cp2 = pix2 - (lp2 << 5);

	pixPosDif = lp1 - lp2;
	if (pixPosDif > -2 && pixPosDif < 2) {
		pixPosDif = cp1 - cp2;
		if (pixPosDif > -2 && pixPosDif < 2) {
			return false;
		}

	}

	return true;
}

float MLX90640::getMedian(float *values, int n) const noexcept {
	float temp;

	for (int i = 0; i < n - 1; i++) {
		for (int j = i + 1; j < n; j++) {
			if (values[j] < values[i]) {
				temp = values[i];
				values[i] = values[j];
				values[j] = temp;
			}
		}
	}

	if (n % 2 == 0) {
		return ((values[n / 2] + values[n / 2 - 1]) / 2.0);

	} else {
		return values[n / 2];
	}
}

bool MLX90640::isPixelBad(uint16_t pixel) const noexcept {
	for (int i = 0; i < 5; i++) {
		if (pixel == params.outlierPixels[i]
				|| pixel == params.brokenPixels[i]) {
			return true;
		}
	}

	return false;
}

bool MLX90640::validateFrameData(const uint16_t *frameData) const noexcept {
	uint8_t line = 0;

	for (unsigned i = 0; i < PIXELS; i += MLX90640_LINE_SIZE)
	{
		if ((frameData[i] == 0x7FFF) && (line % 2 == frameData[833]))
			return false;
		line = line + 1;
	}

	return true;
}

bool MLX90640::validateAuxData(const uint16_t *auxData) const noexcept {
	if (auxData[0] == 0x7FFF)
		return false;

	for (int i = 8; i < 19; i++) {
		if (auxData[i] == 0x7FFF)
			return false;
	}

	for (int i = 20; i < 23; i++) {
		if (auxData[i] == 0x7FFF)
			return false;
	}

	for (int i = 24; i < 33; i++) {
		if (auxData[i] == 0x7FFF)
			return false;
	}

	for (int i = 40; i < 51; i++) {
		if (auxData[i] == 0x7FFF)
			return false;
	}

	for (int i = 52; i < 55; i++) {
		if (auxData[i] == 0x7FFF)
			return false;
	}

	for (int i = 56; i < 64; i++) {
		if (auxData[i] == 0x7FFF)
			return false;
	}

	return true;
}

void MLX90640::badPixelsCorrection(const uint16_t *pixels, MLX90640Mode mode,
		MLX90640Data &target) const noexcept {
	float ap[4];
	uint8_t pix;
	uint8_t line;
	uint8_t column;

	pix = 0;
	auto to = target.data;
	while (pixels[pix] != 0xFFFF) {
		line = pixels[pix] >> 5;
		column = pixels[pix] - (line << 5);

		if (mode == 1) {
			if (line == 0) {
				if (column == 0) {
					to[pixels[pix]] = to[33];
				} else if (column == 31) {
					to[pixels[pix]] = to[62];
				} else {
					to[pixels[pix]] = (to[pixels[pix] + 31]
							+ to[pixels[pix] + 33]) / 2.0;
				}
			} else if (line == 23) {
				if (column == 0) {
					to[pixels[pix]] = to[705];
				} else if (column == 31) {
					to[pixels[pix]] = to[734];
				} else {
					to[pixels[pix]] = (to[pixels[pix] - 33]
							+ to[pixels[pix] - 31]) / 2.0;
				}
			} else if (column == 0) {
				to[pixels[pix]] = (to[pixels[pix] - 31] + to[pixels[pix] + 33])
						/ 2.0;
			} else if (column == 31) {
				to[pixels[pix]] = (to[pixels[pix] - 33] + to[pixels[pix] + 31])
						/ 2.0;
			} else {
				ap[0] = to[pixels[pix] - 33];
				ap[1] = to[pixels[pix] - 31];
				ap[2] = to[pixels[pix] + 31];
				ap[3] = to[pixels[pix] + 33];
				to[pixels[pix]] = getMedian(ap, 4);
			}
		} else {
			if (column == 0) {
				to[pixels[pix]] = to[pixels[pix] + 1];
			} else if (column == 1 || column == 30) {
				to[pixels[pix]] = (to[pixels[pix] - 1] + to[pixels[pix] + 1])
						/ 2.0;
			} else if (column == 31) {
				to[pixels[pix]] = to[pixels[pix] - 1];
			} else {
				if (!isPixelBad(pixels[pix] - 2)
						&& !isPixelBad(pixels[pix] + 2)) {
					ap[0] = to[pixels[pix] + 1] - to[pixels[pix] + 2];
					ap[1] = to[pixels[pix] - 1] - to[pixels[pix] - 2];
					if (fabs(ap[0]) > fabs(ap[1])) {
						to[pixels[pix]] = to[pixels[pix] - 1] + ap[1];
					} else {
						to[pixels[pix]] = to[pixels[pix] + 1] + ap[0];
					}
				} else {
					to[pixels[pix]] =
							(to[pixels[pix] - 1] + to[pixels[pix] + 1]) / 2.0;
				}
			}
		}
		pix = pix + 1;
	}
}

} /* namespace wanhive */
