/*
 * I2C.h
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

#ifndef WH_PHYSICAL_I2C_H_
#define WH_PHYSICAL_I2C_H_
#include <wanhive/base/unix/File.h>

namespace wanhive {
/**
 * I2C device properties
 */
struct I2CDevice {
	/*! Slave address */
	unsigned int address { 0 };
	/*! Enable ten bit address */
	bool tenbit { false };
	/*! Enable packet error checking */
	bool pec { false };
};
/**
 * User space I2C bus driver.
 * @ref https://www.kernel.org/doc/html/latest/i2c/
 */
class I2C: protected File {
public:
	/**
	 * Constructor: opens an i2c adapter (doesn't open a device).
	 * @param bus adapter's identifier
	 */
	I2C(unsigned int bus);
	/**
	 * Constructor: opens an i2c device.
	 * @param bus adapter's identifier
	 * @param address device's 7-bit address
	 */
	I2C(unsigned int bus, unsigned int address);
	/**
	 * Constructor: opens an i2c device.
	 * @param bus adapter's identifier
	 * @param device device's specification
	 */
	I2C(unsigned int bus, const I2CDevice &device);
	/**
	 * Constructor: opens an i2c adapter (doesn't open a device).
	 * @param path adapter's pathname
	 */
	I2C(const char *path);
	/**
	 * Constructor: opens an i2c device.
	 * @param path adapter's pathname
	 * @param address device's 7-bit address
	 */
	I2C(const char *path, unsigned int address);
	/**
	 * Constructor: opens an i2c device.
	 * @param path adapter's pathname
	 * @param device device's properties
	 */
	I2C(const char *path, const I2CDevice &device);
	/**
	 * Destructor
	 */
	~I2C();
	/**
	 * Returns adapter's capabilities.
	 * @return bitmap of adapter's capabilities
	 */
	unsigned long capabilities() const noexcept;
	/**
	 * Selects a device for communication.
	 * @param device device's properties
	 */
	void select(const I2CDevice &device) const;
private:
	void open(const char *path);
	void open(const char *path, const I2CDevice &device);
private:
	unsigned long functions { 0 };
};

} /* namespace wanhive */

#endif /* WH_PHYSICAL_I2C_H_ */
