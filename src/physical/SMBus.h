/*
 * SMBus.h
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

#ifndef WH_PHYSICAL_SMBUS_H_
#define WH_PHYSICAL_SMBUS_H_
#include "I2C.h"

namespace wanhive {
/**
 * User space SMBus driver
 */
class SMBus: protected I2C {
public:
	/**
	 * Constructor: opens a device.
	 * @param bus adapter's identifier
	 * @param address device's 7-bit address
	 */
	SMBus(unsigned int bus, unsigned int address);
	/**
	 * Constructor: opens a device.
	 * @param bus adapter's identifier
	 * @param device device's specification
	 */
	SMBus(unsigned int bus, const I2CDevice &device);
	/**
	 * Constructor: opens a device.
	 * @param path adapter's pathname
	 * @param address device's 7-bit address
	 */
	SMBus(const char *path, unsigned int address);
	/**
	 * Constructor: opens a device.
	 * @param path adapter's pathname
	 * @param device device's specification
	 */
	SMBus(const char *path, const I2CDevice &device);
	/**
	 * Destructor
	 */
	~SMBus();
	//-----------------------------------------------------------------
	/**
	 * Reads a block of up to 32 bytes from a device.
	 * @param command register selector
	 * @param count number of bytes to read
	 * @param buffer stores the incoming data
	 * @return number of bytes read from the device
	 */
	unsigned int read(unsigned char command, unsigned int count,
			void *buffer) const;
	/**
	 * Writes a block of up to 32 bytes to a device.
	 * @param command designated register
	 * @param count number of bytes to write
	 * @param buffer data to write
	 */
	void write(unsigned char command, unsigned int count,
			const void *buffer) const;
	//-----------------------------------------------------------------
	/**
	 * Reads a single byte from a device.
	 * @param command register selector
	 * @param value stores the received value
	 */
	void read(unsigned char command, unsigned char &value) const;
	/**
	 * Reads a single byte from a device.
	 * @param command register selector
	 * @return value received from the device
	 */
	unsigned char readByte(unsigned char command) const;
	/**
	 * Writes a single byte to a device.
	 * @param command designated register
	 * @param value value to write
	 */
	void write(unsigned char command, unsigned char value) const;
	//-----------------------------------------------------------------
	/**
	 * Reads a word (16 bits) from a device.
	 * @param command register selector
	 * @param value stores the received value
	 */
	void read(unsigned char command, unsigned short &value) const;
	/**
	 * Reads a word (16 bits) from a device.
	 * @param command register selector
	 * @return value received from the device
	 */
	unsigned short readWord(unsigned char command) const;
	/**
	 * Writes a word (16 bits) to a device.
	 * @param command designated register
	 * @param value value to write
	 */
	void write(unsigned char command, unsigned short value) const;
	//-----------------------------------------------------------------
	/**
	 * Reads a single byte from a simple device.
	 * @param value stores the received value
	 */
	void read(unsigned char &value) const;
	/**
	 * Reads a single byte from a simple device.
	 * @return value received from the device
	 */
	unsigned char read() const;
	/**
	 * Writes a single byte to a simple device.
	 * @param value value to write
	 */
	void write(unsigned char value) const;
	//-----------------------------------------------------------------
	/**
	 * Quick write (for probing): sends a single bit to a device.
	 * @param value value to write
	 */
	void quickWrite(unsigned char value) const;
	//-----------------------------------------------------------------
	/**
	 * Process call: sends 1 to 31 bytes of data to a device, and reads 1 to 31
	 * bytes of data in return.
	 * @param command register selector
	 * @param count number of bytes to write
	 * @param buffer value-result argument to provide the outgoing data and
	 * store the received data.
	 * @return number of received bytes
	 */
	unsigned int process(unsigned char command, unsigned int count,
			void *buffer) const;
	/**
	 * Process call: sends 16 bits of data to a device, and reads 16 bits of
	 * data in return.
	 * @param command register selector
	 * @param value data to write
	 * @return received data
	 */
	unsigned short process(unsigned char command, unsigned short value) const;
};

} /* namespace wanhive */

#endif /* WH_PHYSICAL_SMBUS_H_ */
