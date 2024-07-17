/*
 * SMBus.cpp
 *
 * Copyright (C) 2023 Amit Kumar (amitkriit@gmail.com)
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

#include "SMBus.h"
#include <wanhive/base/common/Exception.h>
#include <wanhive/base/unix/SystemException.h>
extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

namespace wanhive {

SMBus::SMBus(unsigned int bus, unsigned int address) :
		I2C(bus, address) {

}

SMBus::SMBus(unsigned int bus, const I2CDevice &device) :
		I2C(bus, device) {

}

SMBus::SMBus(const char *path, unsigned int address) :
		I2C(path, address) {

}

SMBus::SMBus(const char *path, const I2CDevice &device) :
		I2C(path, device) {

}

SMBus::~SMBus() {

}

unsigned int SMBus::read(unsigned char command, unsigned int count,
		void *buffer) const {
	if (count > I2C_SMBUS_BLOCK_MAX) {
		throw Exception(EX_OVERFLOW);
	}

	auto rv = i2c_smbus_read_i2c_block_data(File::get(), command, count,
			(unsigned char*) buffer);
	if (rv < 0) {
		throw SystemException();
	} else {
		return rv;
	}
}

void SMBus::write(unsigned char command, unsigned int count,
		const void *buffer) const {
	if (count > I2C_SMBUS_BLOCK_MAX) {
		throw Exception(EX_OVERFLOW);
	}

	auto rv = i2c_smbus_write_i2c_block_data(File::get(), command, count,
			(const unsigned char*) buffer);
	if (rv < 0) {
		throw SystemException();
	}
}

void SMBus::read(unsigned char command, unsigned char &value) const {
	auto rv = i2c_smbus_read_byte_data(File::get(), command);
	if (rv < 0) {
		throw SystemException();
	} else {
		value = rv & 0xFF;
	}
}

unsigned char SMBus::readByte(unsigned char command) const {
	auto rv = i2c_smbus_read_byte_data(File::get(), command);
	if (rv < 0) {
		throw SystemException();
	} else {
		return (rv & 0xFF);
	}
}

void SMBus::write(unsigned char command, unsigned char value) const {
	if (i2c_smbus_write_byte_data(File::get(), command, value) < 0) {
		throw SystemException();
	}
}

void SMBus::read(unsigned char command, unsigned short &value) const {
	auto rv = i2c_smbus_read_word_data(File::get(), command);
	if (rv < 0) {
		throw SystemException();
	} else {
		value = rv & 0xFFFF;
	}
}

unsigned short SMBus::readWord(unsigned char command) const {
	auto rv = i2c_smbus_read_word_data(File::get(), command);
	if (rv < 0) {
		throw SystemException();
	} else {
		return (rv & 0xFFFF);
	}
}

void SMBus::write(unsigned char command, unsigned short value) const {
	if (i2c_smbus_write_word_data(File::get(), command, value) < 0) {
		throw SystemException();
	}
}

void SMBus::read(unsigned char &value) const {
	auto rv = i2c_smbus_read_byte(File::get());
	if (rv < 0) {
		throw SystemException();
	} else {
		value = rv & 0xFF;
	}
}

unsigned char SMBus::read() const {
	auto rv = i2c_smbus_read_byte(File::get());
	if (rv < 0) {
		throw SystemException();
	} else {
		return (rv & 0xFF);
	}
}

void SMBus::write(unsigned char value) const {
	if (i2c_smbus_write_byte(File::get(), value) < 0) {
		throw SystemException();
	}
}

void SMBus::quickWrite(unsigned char value) const {
	if (i2c_smbus_write_quick(File::get(), value) < 0) {
		throw SystemException();
	}
}

unsigned int SMBus::process(unsigned char command, unsigned int count,
		void *buffer) const {
	auto rv = i2c_smbus_block_process_call(File::get(), command, count,
			(unsigned char*) buffer);
	if (rv < 0) {
		throw SystemException();
	} else {
		return rv;
	}
}

unsigned short SMBus::process(unsigned char command,
		unsigned short value) const {
	auto rv = i2c_smbus_process_call(File::get(), command, value);
	if (rv < 0) {
		throw SystemException();
	} else {
		return (rv & 0xFFFF);
	}
}

} /* namespace wanhive */
