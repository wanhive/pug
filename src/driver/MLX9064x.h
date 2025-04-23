/*
 * MLX9064x.h
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

#ifndef WH_DRIVER_MLX9064X_H_
#define WH_DRIVER_MLX9064X_H_
#include "../physical/I2C.h"
#include <cstdint>

namespace wanhive {
/**
 * Bare bone MLX90640/MLX90641 thermal imaging camera driver.
 * @note Provides the common I/O facilities over an I2C interface.
 */
class MLX9064x: protected I2C {
public:
	/**
	 * Constructor: opens the I2C bus.
	 * @param bus i2c adapter's pathname
	 * @param address device identifier
	 */
	MLX9064x(unsigned int bus, unsigned int address);
	/**
	 * Constructor: opens the I2C bus.
	 * @param path i2c adapter's pathname
	 * @param address device identifier
	 */
	MLX9064x(const char *path, unsigned int address);
	/**
	 * Destructor: closes the i2c bus.
	 */
	~MLX9064x();
	/**
	 * Resets the device (uses the general call reset).
	 */
	void reset();
	/**
	 * Reads a word from a given register.
	 * @param command register's identifier
	 * @return register's value
	 */
	uint16_t readReg(uint16_t command) const;
	/**
	 * Reads a given number of words from a register.
	 * @param command register's identifier
	 * @param count number of words to read
	 * @param data words read from the register
	 */
	void readReg(uint16_t command, uint16_t count, uint16_t *data) const;
	/**
	 * Writes a word to a register.
	 * @param command register's identifier
	 * @param data word to write
	 */
	void writeReg(uint16_t command, uint16_t data) const;
private:
	unsigned int address;
};

} /* namespace wanhive */

#endif /* WH_DRIVER_MLX9064X_H_ */
