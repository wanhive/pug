/*
 * UART.cpp
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

#include "UART.h"

namespace {

void setDataBits(wanhive::TerminalConfig &tc, wanhive::FrameBits bits) noexcept {
	tcflag_t flag = 0;
	switch (bits) {
	case wanhive::FrameBits::BITS_5:
		flag = CS5;
		break;
	case wanhive::FrameBits::BITS_6:
		flag = CS6;
		break;
	case wanhive::FrameBits::BITS_7:
		flag = CS7;
		break;
	default:
		flag = CS8;
		break;
	}

	tc.maskFlag(wanhive::TerminalMode::FLOW, ~(CSIZE));
	tc.setFlag(wanhive::TerminalMode::FLOW, flag);
}

void setParity(wanhive::TerminalConfig &tc, wanhive::Parity parity) noexcept {
	switch (parity) {
	case wanhive::Parity::EVEN:
		tc.setFlag(wanhive::TerminalMode::INPUT, INPCK);
		tc.setFlag(wanhive::TerminalMode::FLOW, PARENB);
		tc.maskFlag(wanhive::TerminalMode::FLOW, ~(PARODD));
		break;
	case wanhive::Parity::ODD:
		tc.setFlag(wanhive::TerminalMode::INPUT, INPCK);
		tc.setFlag(wanhive::TerminalMode::FLOW, (PARENB | PARODD));
		break;
	default:
		tc.maskFlag(wanhive::TerminalMode::INPUT, ~(INPCK));
		tc.maskFlag(wanhive::TerminalMode::FLOW, ~(PARENB | PARODD));
		break;
	}
}

void setStopBits(wanhive::TerminalConfig &tc, wanhive::StopBits bits) noexcept {
	switch (bits) {
	case wanhive::StopBits::STOP_2:
		tc.setFlag(wanhive::TerminalMode::FLOW, CSTOPB);
		break;
	default:
		break;
	}
}

void setFlowControl(wanhive::TerminalConfig &tc, bool enable) noexcept {
	if (enable) {
		tc.setFlag(wanhive::TerminalMode::FLOW, CRTSCTS);
	} else {
		tc.maskFlag(wanhive::TerminalMode::FLOW, ~(CRTSCTS));
	}
}

void setBlockingMode(wanhive::TerminalConfig &tc, bool block) {
	tc.setControl(VTIME, 0);
	tc.setControl(VMIN, (block ? 1 : 0));
}

}  // namespace

namespace wanhive {

UART::UART(const char *path, const UARTConfig &cfg) :
		Terminal(path, (O_RDWR | O_NOCTTY | (cfg.block ? 0 : O_NONBLOCK))) {
	configure(cfg);
}

UART::~UART() {

}

void UART::configure(const UARTConfig &cfg) const {
	TerminalConfig tc;
	getAttribute(tc);
	tc.raw();
	tc.setBaud(cfg.baud);
	setDataBits(tc, cfg.bits);
	setParity(tc, cfg.parity);
	setStopBits(tc, cfg.stop);
	setFlowControl(tc, cfg.flow);
	setBlockingMode(tc, cfg.block);
	setAttribute(tc);
}

} /* namespace wanhive */
