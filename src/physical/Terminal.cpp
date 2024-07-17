/*
 * Terminal.cpp
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

#include "Terminal.h"
#include <wanhive/base/common/Exception.h>
#include <wanhive/base/unix/Config.h>
#include <wanhive/base/unix/SystemException.h>
#include <unistd.h>

namespace wanhive {

Terminal::Terminal(int fd, bool release) :
		release { release } {
	try {
		if (test(fd)) {
			File::set(fd);
			getAttribute(original);
		} else {
			throw Exception(EX_ARGUMENT);
		}
	} catch (const BaseException &e) {
		File::release();
		throw;
	}
}

Terminal::Terminal(const char *path, int flags) :
		release { false } {
	try {
		File::open(path, flags);
		if (test(File::get())) {
			getAttribute(original);
		} else {
			throw Exception(EX_ARGUMENT);
		}
	} catch (const BaseException &e) {
		File::close();
		throw;
	}
}

Terminal::~Terminal() {
	try {
		restore();
	} catch (...) {
	}

	if (release) {
		File::release();
	}
}

void Terminal::getAttribute(TerminalConfig &tc) const {
	if (::tcgetattr(File::get(), tc.get()) == -1) {
		throw SystemException();
	}
}

void Terminal::setAttribute(const TerminalConfig &tc, int when) const {
	if (::tcsetattr(File::get(), when, tc.get()) == -1) {
		throw SystemException();
	}

	TerminalConfig current;
	getAttribute(current);
	if (!tc.equal(current)) {
		throw Exception(EX_OPERATION);
	}
}

void Terminal::restore() const {
	setAttribute(original, TCSAFLUSH);
}

void Terminal::deviceName(char *buffer, size_t size) const {
	if (!buffer || !size) {
		throw Exception(EX_ARGUMENT);
	} else if (::ttyname_r(File::get(), buffer, size) == -1) {
		throw SystemException();
	} else {
		return;
	}
}

void Terminal::drain() const {
	if (::tcdrain(File::get()) == -1) {
		throw SystemException();
	}
}

void Terminal::flow(TerminalFlow tf) const {
	int action = -1;
	switch (tf) {
	case TerminalFlow::ENABLE_OUT:
		action = TCOON;
		break;
	case TerminalFlow::DISABLE_OUT:
		action = TCOOFF;
		break;
	case TerminalFlow::SEND_START:
		action = TCION;
		break;
	case TerminalFlow::SEND_STOP:
		action = TCIOFF;
		break;
	default:
		break;
	}

	if (::tcflow(File::get(), action) == -1) {
		throw SystemException();
	}
}

void Terminal::flush(TerminalQueue tl) const {
	auto q = (tl == TerminalQueue::INPUT) ? TCIFLUSH : TCOFLUSH;
	if (::tcflush(File::get(), q) == -1) {
		throw SystemException();
	}
}

void Terminal::flush() const {
	if (::tcflush(File::get(), TCIOFLUSH) == -1) {
		throw SystemException();
	}
}

void Terminal::sendBreak(int duration) const {
	if (::tcsendbreak(File::get(), duration) == -1) {
		throw SystemException();
	}
}

cc_t Terminal::vdisable() const {
	auto ret = Config::path(File::get(), _PC_VDISABLE);
	if (ret == -1) {
		throw Exception(EX_ARGUMENT);
	} else {
		return ret;
	}
}

bool Terminal::test(int fd) noexcept {
	return (::isatty(fd) == 1);
}

} /* namespace wanhive */
