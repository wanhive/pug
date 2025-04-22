/*
 * MLX9064x.cpp
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

#include "MLX9064x.h"
#include <wanhive/base/common/Exception.h>
#include <wanhive/base/unix/SystemException.h>
#include <sys/ioctl.h>
extern "C" {
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
}

namespace wanhive {

MLX9064x::MLX9064x(unsigned int bus, unsigned int address) :
		I2C(bus), address { address } {
}

MLX9064x::MLX9064x(const char *path, unsigned int address) :
		I2C(path), address { address } {
}

MLX9064x::~MLX9064x() {

}

void MLX9064x::reset() {
	I2C::resetAll();
}

uint16_t MLX9064x::readReg(uint16_t command) const {
	uint16_t data = 0;
	readReg(command, 1, &data);
	return data;
}

void MLX9064x::readReg(uint16_t command, uint16_t count, uint16_t *data) const {
	constexpr unsigned MAX_BYTES = 4096;
	unsigned int bytes = count * 2;

	if (!bytes) {
		return;
	}

	if ((bytes && !data) || bytes > MAX_BYTES) {
		throw Exception(EX_ARGUMENT);
	}

	i2c_rdwr_ioctl_data iodata;
	i2c_msg msgs[2];
	uint8_t cmd[2] = { (uint8_t) (command >> 8), (uint8_t) (command & 0xFF) };
	msgs[0].addr = address;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = cmd;

	msgs[1].addr = address;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = bytes;
	msgs[1].buf = (uint8_t*) data;

	iodata.msgs = msgs;
	iodata.nmsgs = 2;

	if (::ioctl(File::get(), I2C_RDWR, &iodata) == -1) {
		throw SystemException();
	}

	for (unsigned i = 0; i < count; ++i) {
		auto raw = (uint8_t*) data;
		auto index = (i << 1);
		data[i] = (uint16_t) ((raw[index] << 8) | raw[index + 1]);
	}
}

void MLX9064x::writeReg(uint16_t command, uint16_t data) const {
	i2c_rdwr_ioctl_data iodata;
	i2c_msg msg;

	uint8_t cmd[8] = { (uint8_t) (command >> 8), (uint8_t) (command & 0xFF),
			(uint8_t) (data >> 8), (uint8_t) (data & 0xFF) };
	msg.addr = address;
	msg.flags = 0;
	msg.len = 4;
	msg.buf = cmd;

	iodata.msgs = &msg;
	iodata.nmsgs = 1;

	if (::ioctl(File::get(), I2C_RDWR, &iodata) == -1) {
		throw SystemException();
	}
}

} /* namespace wanhive */
