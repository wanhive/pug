/**
 * @file UART.h
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
#include "UARTConfig.h"

/*! @namespace wanhive */
namespace wanhive {
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
