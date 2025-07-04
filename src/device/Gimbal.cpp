/*
 * Gimbal.cpp
 *
 * Copyright (C) 2024 Wanhive Systems Private Limited (info@wanhive.com)
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

#include "Gimbal.h"

namespace wanhive {

Gimbal::Gimbal(unsigned int bus, unsigned int address) :
		PWM(PWM::FREQUENCY, bus, address) {
}

Gimbal::Gimbal(const char *path, unsigned int address) :
		PWM(PWM::FREQUENCY, path, address) {
}

Gimbal::~Gimbal() {

}

void Gimbal::pan(unsigned int value) {
	if (value != axis.pan && value <= PAN_MAX) {
		servo(CTRL_PAN, pulseWidth(_pan, value));
		axis.pan = value;
	}
}

void Gimbal::roll(unsigned int value) {
	if (value != axis.roll && value <= ROLL_MAX) {
		servo(CTRL_ROLL, pulseWidth(_roll, value));
		axis.roll = value;
	}
}

void Gimbal::tilt(unsigned int value) {
	if (value != axis.tilt && value <= TILT_MAX) {
		servo(CTRL_TILT, pulseWidth(_tilt, value));
		axis.tilt = value;
	}
}

void Gimbal::reset() {
	pan(CENTER);
	roll(CENTER);
	tilt(CENTER);
	alert(false);
}

void Gimbal::alert(bool on) const {
	if (on) {
		high(CTRL_ALERT);
	} else {
		low(CTRL_ALERT);
	}
}

unsigned int Gimbal::pulseWidth(const ServoControl &sc,
		unsigned int angle) const noexcept {
	return (sc.pulse(angle - 90.0) + 0.5);
}

void Gimbal::configure(unsigned int center, double slope) noexcept {
	_pan.set(center, slope);
	_roll.set(center, slope);
	_tilt.set(center, slope);
}

} /* namespace wanhive */
