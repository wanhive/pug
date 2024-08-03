/*
 * ADS111x.cpp
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

#include "ADS111x.h"
#include <endian.h>

namespace {

constexpr unsigned char CONVERSION_REG = 0x00; //Conversion register
constexpr unsigned char CONFIG_REG = 0x01; //Config register
constexpr unsigned char LO_THRESHOLD_REG = 0x02; //Lo_thresh register
constexpr unsigned char HI_THRESHOLD_REG = 0x03; //Hi_thresh register

}  // namespace

namespace wanhive {

ADS111x::ADS111x(unsigned int bus, unsigned int address) :
		SMBus(bus, address) {

}

ADS111x::ADS111x(const char *path, unsigned int address) :
		SMBus(path, address) {

}

ADS111x::~ADS111x() {

}

unsigned short ADS111x::getConversion() const {
	return be16toh(SMBus::readWord(CONVERSION_REG));
}

unsigned short ADS111x::getConfiguration() const {
	return be16toh(SMBus::readWord(CONFIG_REG));
}

void ADS111x::setConfiguration(unsigned short value) const {
	SMBus::write(CONFIG_REG, htobe16(value));
}

unsigned short ADS111x::getLowThreshold() const {
	return be16toh(SMBus::readWord(LO_THRESHOLD_REG));
}

void ADS111x::setLowThreshold(unsigned short value) const {
	SMBus::write(LO_THRESHOLD_REG, htobe16(value));
}

unsigned short ADS111x::getHighThreshold() const {
	return be16toh(SMBus::readWord(HI_THRESHOLD_REG));
}

void ADS111x::setHighThreshold(unsigned short value) const {
	SMBus::write(HI_THRESHOLD_REG, htobe16(value));
}

void ADS111x::decode(unsigned short value, ADS111xConfig &config) noexcept {
	config.queue = value & 0b11;
	config.latch = (value >> 2) & 0b1;
	config.polarity = (value >> 3) & 0b1;
	config.cmode = (value >> 4) & 0b1;
	config.rate = (value >> 5) & 0b111;
	config.mode = (value >> 8) & 0b1;
	config.gain = (value >> 9) & 0b111;
	config.mux = (value >> 12) & 0b111;
	config.status = (value >> 15) & 0b1;
}

unsigned short ADS111x::encode(const ADS111xConfig &config) noexcept {
	return ((config.queue) | (config.latch << 2) | (config.polarity << 3)
			| (config.cmode << 4) | (config.rate << 5) | (config.mode << 8)
			| (config.gain << 9) | (config.mux << 12) | (config.status << 15));

}

} /* namespace wanhive */
