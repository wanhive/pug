/*
 * UltraSonic.h
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

#ifndef WH_DEVICE_ULTRASONIC_H_
#define WH_DEVICE_ULTRASONIC_H_
#include <wanhive/base/common/Source.h>
#include <wanhive/base/ds/Mean.h>

namespace wanhive {
/**
 * Ultrasonic sensor driver.
 * @note Supports the commonly used UART-auto data format
 */
class UltraSonic {
public:
	/**
	 * Constructor: initializes the driver.
	 */
	UltraSonic() noexcept;
	/**
	 * Destructor
	 */
	~UltraSonic();
	/**
	 * Processes the incoming sensor data.
	 * @param source data source
	 */
	void process(Source<unsigned char> &source) noexcept;
	/**
	 * Checks if sufficient data is available.
	 * @param threshold minimum observations count
	 * @return true on sufficient data, false otherwise
	 */
	bool available(unsigned int threshold = 0) const noexcept;
	/**
	 * Returns the average of the accumulated data.
	 * @param threshold minimum observations count
	 * @return average value on success, zero (0) on failure
	 */
	unsigned int average(unsigned int threshold = 0) const noexcept;
	/**
	 * Resets the driver to its initial state.
	 */
	void reset() noexcept;
	/**
	 * Clears the accumulator.
	 */
	void clear() noexcept;
private:
	void accumulate(unsigned char cs) noexcept;
private:
	struct {
		unsigned char state;
		unsigned char data[2];
	} ctx;

	Mean<unsigned long long> accumulator;
};

} /* namespace wanhive */

#endif /* WH_DEVICE_ULTRASONIC_H_ */
