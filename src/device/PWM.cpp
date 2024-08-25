/*
 * PWM.cpp
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

#include "PWM.h"
#include <wanhive/base/common/Exception.h>

#ifndef WH_PWM_FREQUENCY
#define WH_PWM_FREQUENCY 50U
#endif
#ifndef WH_SERVO_NEUTRAL_PULSE
#define WH_SERVO_NEUTRAL_PULSE 1.5f
#endif

namespace wanhive {

const unsigned int PWM::FREQUENCY = WH_PWM_FREQUENCY;
const float PWM::SERVO_CENTER = WH_SERVO_NEUTRAL_PULSE;

PWM::PWM(unsigned int bus, unsigned int address) :
		PCA9685(bus, address) {
	setFrequency(FREQUENCY);
}

PWM::PWM(const char *path, unsigned int address) :
		PCA9685(path, address) {
	setFrequency(FREQUENCY);
}

PWM::~PWM() {

}

void PWM::servo(unsigned int pin, float pulse) const {
	int value = ((PWM_MAX * (pulse * FREQUENCY / 1000.0f)) + 0.5f);
	value = (value >= 0) ? value : 0;
	pwmWrite(pin, value);
}

void PWM::high(unsigned int pin) const {
	digitalWrite(pin, true);
}

void PWM::low(unsigned int pin) const {
	digitalWrite(pin, false);
}

void PWM::pulse(unsigned int pin, unsigned int delay, unsigned int duty) const {
	if (delay > 100 || duty > 100) {
		throw Exception(EX_ARGUMENT);
	}

	unsigned short on = ((PWM_MAX * (delay / 100.0f)) + 0.5f);
	unsigned short high = ((PWM_MAX * (duty / 100.0f)) + 0.5f);
	unsigned short total = (on + high);
	unsigned short off = (total <= PWM_MAX) ? total : (total - PWM_MAX);

	on = (on > 0) ? (on - 1) : on;
	off = (off > 0) ? (off - 1) : off;
	PCA9685::write(pin, on, off);
}

} /* namespace wanhive */
