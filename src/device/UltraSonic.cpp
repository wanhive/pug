/*
 * UltraSonic.cpp
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

#include "UltraSonic.h"
#include <cstring>

namespace {

enum UState : unsigned char {
	US_HDR = 0, US_H = 1, US_L = 2, US_CS = 3
};

constexpr unsigned char US_START = 0xff;

}  // namespace

namespace wanhive {

UltraSonic::UltraSonic() noexcept {
	reset();
}

UltraSonic::~UltraSonic() {

}

void UltraSonic::process(Source<unsigned char> &source) noexcept {
	unsigned char next = 0;
	while (source.emit(next)) {
		switch (ctx.state) {
		case US_HDR:
			if (next == US_START) {
				ctx.state = US_H;
			}
			break;
		case US_H:
			ctx.data[0] = next;
			ctx.state = US_L;
			break;
		case US_L:
			ctx.data[1] = next;
			ctx.state = US_CS;
			break;
		case US_CS:
			accumulate(next);
			ctx.state = US_HDR;
			break;
		default:
			ctx.state = US_HDR;
			break;
		}
	}
}

bool UltraSonic::available(unsigned int threshold) const noexcept {
	return accumulator.available(threshold);
}

unsigned int UltraSonic::average(unsigned int threshold) const noexcept {
	unsigned long long value = 0;
	accumulator.calculate(value, threshold);
	return value;
}

void UltraSonic::reset() noexcept {
	memset(&ctx, 0, sizeof(ctx));
	accumulator.clear();
}

void UltraSonic::clear() noexcept {
	accumulator.clear();
}

void UltraSonic::accumulate(unsigned char cs) noexcept {
	auto high = ctx.data[0];
	auto low = ctx.data[1];
	if (((US_START + high + low) & 0xff) == cs) {
		accumulator.accumulate(high * 256 + low);
	}
}

} /* namespace wanhive */
