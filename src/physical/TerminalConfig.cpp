/*
 * TerminalConfig.cpp
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

#include "TerminalConfig.h"
#include <wanhive/base/common/Exception.h>
#include <wanhive/base/unix/SystemException.h>
#include <cstring>

namespace wanhive {

TerminalConfig::TerminalConfig() {
	::memset(&cfg, 0, sizeof(cfg));
}

TerminalConfig::~TerminalConfig() {

}

void TerminalConfig::raw() noexcept {
	::cfmakeraw(&cfg);
}

tcflag_t TerminalConfig::getFlag(TerminalMode tm) const noexcept {
	switch (tm) {
	case TerminalMode::INPUT:
		return cfg.c_iflag;
	case TerminalMode::OUTPUT:
		return cfg.c_oflag;
	case TerminalMode::FLOW:
		return cfg.c_cflag;
	case TerminalMode::LOCAL:
		return cfg.c_lflag;
	default:
		return 0;
	}
}

void TerminalConfig::setFlag(TerminalMode tm, tcflag_t flags) noexcept {
	switch (tm) {
	case TerminalMode::INPUT:
		cfg.c_iflag |= flags;
		break;
	case TerminalMode::OUTPUT:
		cfg.c_oflag |= flags;
		break;
	case TerminalMode::FLOW:
		cfg.c_cflag |= flags;
		break;
	case TerminalMode::LOCAL:
		cfg.c_lflag |= flags;
		break;
	default:
		break;
	}
}

void TerminalConfig::maskFlag(TerminalMode tm, tcflag_t flags) noexcept {
	switch (tm) {
	case TerminalMode::INPUT:
		cfg.c_iflag &= flags;
		break;
	case TerminalMode::OUTPUT:
		cfg.c_oflag &= flags;
		break;
	case TerminalMode::FLOW:
		cfg.c_cflag &= flags;
		break;
	case TerminalMode::LOCAL:
		cfg.c_lflag &= flags;
		break;
	default:
		break;
	}
}

cc_t TerminalConfig::getControl(unsigned int option) const noexcept {
	if (option < NCCS) {
		return cfg.c_cc[option];
	} else {
		return 0;
	}
}

void TerminalConfig::setControl(unsigned int option, cc_t value) noexcept {
	if (option < NCCS) {
		cfg.c_cc[option] = value;
	}
}

speed_t TerminalConfig::getBaud(TerminalQueue tq) noexcept {
	switch (tq) {
	case TerminalQueue::INPUT:
		return ::cfgetispeed(&cfg);
	case TerminalQueue::OUTPUT:
		return ::cfgetospeed(&cfg);
	default:
		return B0;
	}
}

void TerminalConfig::setBaud(TerminalQueue tq, speed_t baud) {
	switch (tq) {
	case TerminalQueue::INPUT:
		if (::cfsetispeed(&cfg, baud) == -1) {
			throw SystemException();
		}
		break;
	case TerminalQueue::OUTPUT:
		if (::cfsetospeed(&cfg, baud) == -1) {
			throw SystemException();
		}
		break;
	default:
		throw Exception(EX_ARGUMENT);
	}
}

void TerminalConfig::setBaud(speed_t baud) {
	if (::cfsetspeed(&cfg, baud) == -1) {
		throw SystemException();
	}
}

bool TerminalConfig::equal(const TerminalConfig &tc) const noexcept {
	if (cfg.c_cflag != tc.cfg.c_cflag) {
		return false;
	} else if (cfg.c_iflag != tc.cfg.c_iflag) {
		return false;
	} else if (cfg.c_ispeed != tc.cfg.c_ispeed) {
		return false;
	} else if (cfg.c_lflag != tc.cfg.c_lflag) {
		return false;
	} else if (cfg.c_oflag != tc.cfg.c_oflag) {
		return false;
	} else if (cfg.c_ospeed != tc.cfg.c_ospeed) {
		return false;
	} else if (::memcmp(cfg.c_cc, tc.cfg.c_cc, sizeof(cfg.c_cc)) != 0) {
		return false;
	} else {
		return true;
	}
}

termios* TerminalConfig::get() noexcept {
	return &cfg;
}

const termios* TerminalConfig::get() const noexcept {
	return &cfg;
}

} /* namespace wanhive */
