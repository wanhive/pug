/*
 * TerminalConfig.h
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

#ifndef WH_PHYSICAL_TERMINALCONFIG_H_
#define WH_PHYSICAL_TERMINALCONFIG_H_
#include <termios.h>

namespace wanhive {
/**
 * Possible terminal configuration modes
 */
enum class TerminalMode {
	INPUT, /**< Input mode */
	OUTPUT,/**< Output mode */
	FLOW, /**< Flow control mode */
	LOCAL, /**< Local mode */
};

/**
 * Possible terminal IO queues
 */
enum class TerminalQueue {
	INPUT, /**< Input queue */
	OUTPUT /**< Output queue */
};

/**
 * Possible software flow control (XON/XOFF) options
 */
enum class TerminalFlow {
	ENABLE_OUT, /**< Restart suspended output */
	DISABLE_OUT,/**< Suspend the output */
	SEND_START, /**< Transmit a start character */
	SEND_STOP /**< Transmit a stop character */
};

/**
 * Terminal configuration manager
 */
class TerminalConfig {
public:
	/**
	 * Default constructor: zeroes out the configuration data.
	 */
	TerminalConfig();
	/**
	 * Destructor
	 */
	~TerminalConfig();
	//-----------------------------------------------------------------
	/**
	 * Wrapper for cfmakeraw(3): creates "raw mode" configuration.
	 */
	void raw() noexcept;
	//-----------------------------------------------------------------
	/**
	 * Returns configuration flags.
	 * @param tm configuration mode selector
	 * @return configuration flag bitmap
	 */
	tcflag_t getFlag(TerminalMode tm) const noexcept;
	/**
	 * Sets (bitwise-OR) configuration flags.
	 * @param tm configuration mode selector
	 * @param flags flag bits to set
	 */
	void setFlag(TerminalMode tm, tcflag_t flags) noexcept;
	/**
	 * Masks (bitwise-AND) configuration flags.
	 * @param tm configuration mode selector
	 * @param flags flag bits to mask
	 */
	void maskFlag(TerminalMode tm, tcflag_t flags) noexcept;
	/**
	 * Returns a control character.
	 * @param option control character's name
	 * @return control character's value
	 */
	cc_t getControl(unsigned int option) const noexcept;
	/**
	 * Sets a control character.
	 * @param option control character's name
	 * @param value control character's new value
	 */
	void setControl(unsigned int option, cc_t value) noexcept;
	//-----------------------------------------------------------------
	/**
	 * Wrapper for cfgetispeed(3) and cfgetospeed(3): returns the baud rate.
	 * @param tq queue selector
	 * @return baud rate
	 */
	speed_t getBaud(TerminalQueue tq) noexcept;
	/**
	 * Wrapper for cfsetispeed(3) and cfsetospeed(3): sets new baud rate.
	 * @param tq queue selector
	 * @param baud new baud rate
	 */
	void setBaud(TerminalQueue tq, speed_t baud);
	/**
	 * Wrapper for cfsetspeed(3): sets new baud rate for both queues.
	 * @param baud new baud rate
	 */
	void setBaud(speed_t baud);
	//-----------------------------------------------------------------
	/**
	 * Compares configurations (only POSIX.1 fields are compared).
	 * @param tc the other configuration
	 * @return true if equal, false if not equal
	 */
	bool equal(const TerminalConfig &tc) const noexcept;
	//-----------------------------------------------------------------
	/**
	 * Returns a pointer to the internal structure.
	 * @return pointer to termios structure
	 */
	termios* get() noexcept;
	/**
	 * Returns a constant pointer to the internal structure.
	 * @return pointer to termios structure
	 */
	const termios* get() const noexcept;
private:
	termios cfg;
};

} /* namespace wanhive */

#endif /* WH_PHYSICAL_TERMINALCONFIG_H_ */
