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
		PWM(bus, address) {
}

Gimbal::Gimbal(const char *path, unsigned int address) :
		PWM(path, address) {

}

Gimbal::~Gimbal() {

}

void Gimbal::pan(unsigned int value) {
	if (value != axis.pan && value <= PAN_MAX) {
		servo(PAN_CTRL, ((value / 90.0f) + 0.5f));
		axis.pan = value;
	}
}

void Gimbal::roll(unsigned int value) {
	if (value != axis.roll && value <= ROLL_MAX) {
		servo(ROLL_CTRL, ((value / 90.0f) + 0.5f));
		axis.roll = value;
	}
}

void Gimbal::tilt(unsigned int value) {
	if (value != axis.tilt && value <= TILT_MAX) {
		servo(TILT_CTRL, ((value / 90.0f) + 0.5f));
		axis.tilt = value;
	}
}

void Gimbal::center() {
	pan(90);
	roll(90);
	tilt(90);
}

void Gimbal::alert(bool on) const {
	if (on) {
		high(ALERT_CTRL);
	} else {
		low(ALERT_CTRL);
	}
}

} /* namespace wanhive */
