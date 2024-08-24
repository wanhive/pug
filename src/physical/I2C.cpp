/*
 * I2C.cpp
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

#include "I2C.h"
#include <wanhive/base/common/Exception.h>
#include <wanhive/base/unix/FStat.h>
#include <wanhive/base/unix/SystemException.h>
#include <sys/ioctl.h>
extern "C" {
#include <linux/i2c-dev.h>
}
#include<cstdio>

namespace {

constexpr unsigned int I2C_MAJOR = 89;

const char* pathName(unsigned id, char *buffer, unsigned size) {
	auto ret = snprintf(buffer, size, "/dev/i2c-%u", id);
	if (ret > 0 && ((unsigned) ret < size)) {
		return buffer;
	} else {
		throw wanhive::Exception(wanhive::EX_OVERFLOW);
	}
}

}  // namespace

namespace wanhive {

I2C::I2C(unsigned int bus) {
	char path[64];
	open(pathName(bus, path, sizeof(path)));
}

I2C::I2C(unsigned int bus, unsigned int address) {
	char buf[64];
	open(pathName(bus, buf, sizeof(buf)), { address, false, false });
}

I2C::I2C(unsigned int bus, const I2CDevice &device) {
	char buf[64];
	open(pathName(bus, buf, sizeof(buf)), device);
}

I2C::I2C(const char *path) {
	open(path);
}

I2C::I2C(const char *path, unsigned int address) {
	open(path, { address, false, false });
}

I2C::I2C(const char *path, const I2CDevice &device) {
	open(path, device);
}

I2C::~I2C() {

}

unsigned long I2C::capabilities() const noexcept {
	return functions;
}

void I2C::select(unsigned int address) const {
	select( { address, false, false });
}

void I2C::select(const I2CDevice &device) const {
	if (::ioctl(File::get(), I2C_TENBIT, (device.tenbit ? 1 : 0)) == -1) {
		throw SystemException();
	} else if (::ioctl(File::get(), I2C_PEC, (device.pec ? 1 : 0)) == -1) {
		throw SystemException();
	} else if (::ioctl(File::get(), I2C_SLAVE, device) == -1) {
		throw SystemException();
	} else {
		//success
	}
}

void I2C::open(const char *path) {
	try {
		File::open(path, O_RDWR);

		FStat stat(File::get());
		if (!(stat.isCharSpecialFile() && stat.majorDeviceId() == I2C_MAJOR)) {
			throw Exception(EX_OPERATION);
		}

		if (::ioctl(File::get(), I2C_FUNCS, &functions) == -1) {
			throw SystemException();
		}
	} catch (const BaseException &e) {
		File::close();
		throw;
	}
}

void I2C::open(const char *path, const I2CDevice &device) {
	try {
		open(path);
		select(device);
	} catch (const BaseException &e) {
		File::close();
		throw;
	}
}

} /* namespace wanhive */
