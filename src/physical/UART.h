/*
 * UART.h
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

#ifndef WH_PHYSICAL_UART_H_
#define WH_PHYSICAL_UART_H_
#include "Terminal.h"

namespace wanhive {
/**
 * Possible data frame size in bits
 */
enum class FrameBits {
	BITS_5,/**< 5 bits */
	BITS_6,/**< 6 bits */
	BITS_7,/**< 7 bits */
	BITS_8 /**< 8 bits */
};
/**
 * Possible parity configuration
 */
enum class Parity {
	EVEN,/**< Even parity */
	ODD, /**< Odd parity */
	NONE /**< Disabled */
};

/**
 * Possible stop bits count
 */
enum class StopBits {
	STOP_1,/**< One stop bit */
	STOP_2 /**< Two stop bits */
};

/**
 * UART configuration data
 */
struct UARTConfig {
	/*! Baud rate */
	speed_t baud { B9600 };
	/*! Data bits */
	FrameBits bits { FrameBits::BITS_8 };
	/*! Parity */
	Parity parity { Parity::NONE };
	/*! Stop bits */
	StopBits stop { StopBits::STOP_1 };
	/*! Hardware flow control */
	bool flow { false };
	/*! Blocking IO */
	bool block { true };
};

/**
 * UART (universal asynchronous receiver-transmitter) driver.
 */
class UART: protected Terminal {
public:
	/**
	 * Constructor: opens an UART device.
	 * @param path device's pathname
	 * @param cfg device's configuration data
	 */
	UART(const char *path, const UARTConfig &cfg);
	/**
	 * Destructor: restores the device's configuration.
	 */
	~UART();
private:
	void configure(const UARTConfig &cfg) const;
};

} /* namespace wanhive */

#endif /* WH_PHYSICAL_UART_H_ */
