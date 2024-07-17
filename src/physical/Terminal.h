/*
 * Terminal.h
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

#ifndef WH_PHYSICAL_TERMINAL_H_
#define WH_PHYSICAL_TERMINAL_H_
#include "TerminalConfig.h"
#include <wanhive/base/unix/File.h>

namespace wanhive {
/**
 * Terminal IO
 */
class Terminal: protected File {
public:
	/**
	 * Constructor: assigns a terminal file descriptor.
	 * @param fd terminal file descriptor
	 * @param release true if the file descriptor should not be closed when the
	 * destructor is called, false otherwise.
	 */
	Terminal(int fd, bool release = true);
	/**
	 * Constructor: opens a terminal device.
	 * @param path terminal device's pathname
	 * @param flags operation flags (see File::open())
	 */
	Terminal(const char *path, int flags = (O_RDWR | O_NOCTTY));
	/**
	 * Destructor: restores the device's configuration and closes the associated
	 * file descriptor.
	 */
	~Terminal();
	//-----------------------------------------------------------------
	/**
	 * Wrapper for tcgetattr(3): Reads terminal device's configuration.
	 * @param tc stores the configuration data
	 */
	void getAttribute(TerminalConfig &tc) const;
	/**
	 * Wrapper for tcsetattr(3): sets terminal device's configuration.
	 * @param tc new configuration data
	 * @param when optional action
	 */
	void setAttribute(const TerminalConfig &tc, int when = TCSANOW) const;
	/**
	 * Restores the terminal device's configuration.
	 */
	void restore() const;
	/**
	 * Wrapper for ttyname_r(3): Returns the terminal device's name.
	 * @param buffer stores the null-terminated device name
	 * @param size buffer's size in bytes
	 */
	void deviceName(char *buffer, size_t size) const;
	//-----------------------------------------------------------------
	/**
	 * Wrapper for tcdrain(3): waits for all output to be transmitted.
	 */
	void drain() const;
	/**
	 * Wrapper for tcflow(3): suspend or restart the transmission or reception.
	 * @param tf flow selector
	 */
	void flow(TerminalFlow tf) const;
	/**
	 * Wrapper for tcflush(3): discards data.
	 * @param tl queue selector
	 */
	void flush(TerminalQueue tl) const;
	/**
	 * Wrapper for tcflush(3): discards both input and output data.
	 */
	void flush() const;
	/**
	 * Wrapper for tcsendbreak(3): sends break for a specific duration.
	 * @param duration duration specifier
	 */
	void sendBreak(int duration = 0) const;
	/**
	 * Returns the _PC_VDISABLE option's value for the terminal device.
	 * @return _PC_VDISABLE value
	 */
	cc_t vdisable() const;
	//-----------------------------------------------------------------
	/**
	 * Wrapper for isatty(3): tests whether the given file descriptor refers
	 * to a terminal.
	 * @param fd open file descriptor
	 * @return true if terminal, false otherwise
	 */
	static bool test(int fd) noexcept;
private:
	const bool release;
	TerminalConfig original;
};

} /* namespace wanhive */

#endif /* WH_PHYSICAL_TERMINAL_H_ */
