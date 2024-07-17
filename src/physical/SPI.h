/*
 * SPI.h
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

#ifndef WH_PHYSICAL_SPI_H_
#define WH_PHYSICAL_SPI_H_
#include <wanhive/base/unix/File.h>

namespace wanhive {
/**
 * Possible SPI clocking configurations
 */
enum class SPIMode {
	MODE_0,/**< Polarity: 0; Phase: 0 */
	MODE_1,/**< Polarity: 0; Phase: 1 */
	MODE_2,/**< Polarity: 1; Phase: 0 */
	MODE_3 /**< Polarity: 1; Phase: 1 */
};

/**
 * SPI device configuration
 */
struct SPIConfig {
	SPIMode mode { SPIMode::MODE_0 }; //Clock configuration
	unsigned long flags { 0 }; //Additional flags
	unsigned long speed { 5000000 }; //Maximum speed (HZ)
	unsigned int bits { 8 }; //Bits in a word
};
/**
 * User space SPI bus driver
 */
class SPI: protected File {
public:
	/**
	 * Constructor: opens SPI device.
	 * @param bus bus identifier
	 * @param device chip selector
	 */
	SPI(unsigned int bus, unsigned int device);
	/**
	 * Constructor: opens SPI device.
	 * @param bus bus identifier
	 * @param device chip selector
	 * @param cfg configuration data
	 */
	SPI(unsigned int bus, unsigned int device, const SPIConfig &cfg);
	/**
	 * Constructor: opens SPI device.
	 * @param path adapter's pathname
	 */
	SPI(const char *path);
	/**
	 * Constructor: opens SPI device.
	 * @param path adapter's pathname
	 * @param cfg configuration data
	 */
	SPI(const char *path, const SPIConfig &cfg);
	/**
	 * Destructor
	 */
	~SPI();
	//-----------------------------------------------------------------
	/**
	 * Writes data to a device and then reads data in return.
	 * @param tx data to write
	 * @param txBytes number of bytes to writer
	 * @param rx incoming data buffer
	 * @param rxBytes number of bytes to read
	 */
	void transfer(const void *tx, unsigned int txBytes, void *rx,
			unsigned int rxBytes) const;
	/**
	 * Writes data to a device and reads data from it simultaneously. Send and
	 * receive buffers can be the same.
	 * @param tx data to write
	 * @param rx incoming data buffer
	 * @param bytes number of bytes to transfer
	 */
	void transfer(const void *tx, void *rx, unsigned int bytes) const;
	//-----------------------------------------------------------------
	/**
	 * Returns adapter's configuration.
	 * @param config stores the configuration data
	 */
	void getConfiguration(SPIConfig &config) const;
	/**
	 * Updates adapter's configuration.
	 * @param config new configuration data
	 */
	void setConfiguration(const SPIConfig &config) const;
private:
	void open(const char *path);
	void open(const char *path, const SPIConfig &cfg);
};

} /* namespace wanhive */

#endif /* WH_PHYSICAL_SPI_H_ */
