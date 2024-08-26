/*
 * GPIO.h
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

#ifndef WH_PHYSICAL_GPIO_H_
#define WH_PHYSICAL_GPIO_H_
#include "LineConfig.h"
#include <wanhive/base/ds/Buffer.h>
#include <wanhive/base/unix/File.h>

namespace wanhive {
/**
 * GPIO interface for user space.
 */
class GPIO {
public:
	/**
	 * Default constructor: doesn't open a gpio device.
	 */
	GPIO() noexcept;
	/**
	 * Constructor: opens a gpio character special device.
	 * @param path pathname of the gpio device
	 */
	GPIO(const char *path);
	/**
	 * Constructor: opens a gpio character special device.
	 * @param id device's identifier
	 */
	GPIO(unsigned int id);
	/**
	 * Destructor: closes all the file descriptors.
	 */
	~GPIO();
	//-----------------------------------------------------------------
	/**
	 * Opens a gpio character special device.
	 * @param path pathname of the gpio device
	 */
	void open(const char *path);
	/**
	 * Opens a gpio character special device.
	 * @param id device's identifier
	 */
	void open(unsigned int id);
	/**
	 * Closes the open file descriptors associated with the gpio device and the
	 * gpio lines (see GPIO::open() and GPIO::openLine()).
	 */
	void close() noexcept;
	/**
	 * Returns the number of lines associated with the open gpio device.
	 * @return lines count
	 */
	unsigned int getLinesCount() const noexcept;
	/**
	 * Fetches a gpio line's configuration (doesn't open the line).
	 * @param offset index of the gpio line
	 * @param config object for storing the line configuration information
	 */
	void getLineConfiguration(unsigned int offset, LineConfig &config) const;
	//-----------------------------------------------------------------
	/**
	 * Opens a gpio line (closes and replaces any existing descriptor on
	 * success). Call GPIO::closeLine() to close a line.
	 * @param offset gpio line's index
	 * @param config gpio line's configuration
	 * @return anonymous file handle of the gpio line
	 */
	int openLine(unsigned int offset, const LineConfig &config);
	/**
	 * Updates an open gpio line's configuration.
	 * @param offset gpio line's index
	 * @param config the new configuration
	 */
	void configureLine(unsigned int offset, const LineConfig &config) const;
	/**
	 * Closes a gpio line.
	 * @param offset gpio line's index
	 */
	void closeLine(unsigned int offset) noexcept;
	/**
	 * Closes all the open gpio lines.
	 */
	void closeLines() noexcept;
	/**
	 * Returns file descriptor associated with the given gpio line. Do not close
	 * the descriptor with close() system call, use GPIO::closeLine() instead.
	 * @param offset gpio line's index
	 * @return anonymous file handle of the gpio line, -1 on error
	 */
	int getLine(unsigned int offset) const noexcept;
	/**
	 * Reads an open gpio line's status.
	 * @param offset gpio line's index
	 * @return true if the line is active, false otherwise
	 */
	bool read(unsigned int offset) const;
	/**
	 * Reads a GPIO line event.
	 * @param offset gpio line's offset
	 * @param event stores the reported event
	 * @param ts stores the timestamp
	 */
	void readEvent(unsigned int offset, LineEvent &event,
			unsigned long long &timestamp) const;
	/**
	 * Writes an open gpio line's status.
	 * @param offset gpio line's index
	 * @param active true for active, false for inactive
	 */
	void write(unsigned int offset, bool active) const;
	//-----------------------------------------------------------------
	/**
	 * For debugging: prints device's information.
	 */
	void printInfo() const noexcept;
	/**
	 * For debugging: prints the line configuration.
	 * @param config the line configuration
	 */
	static void printConfiguration(const LineConfig &config) noexcept;
private:
	bool setLine(int fd, unsigned int offset) noexcept;
	void resetLines(unsigned int count);
	unsigned int readLinesCount();
private:
	File chip;
	Buffer<int> lines;
};

} /* namespace wanhive */

#endif /* WH_PHYSICAL_GPIO_H_ */
