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

namespace wanhive {

PWM::PWM(unsigned int frequency, unsigned int bus, unsigned int address) :
		PCA9685 { bus, address }, _frequency { setFrequency(frequency) } {

}

PWM::PWM(unsigned int frequency, const char *path, unsigned int address) :
		PCA9685 { path, address }, _frequency { setFrequency(frequency) } {

}

PWM::~PWM() {

}

void PWM::servo(unsigned int pin, unsigned int value) const {
	unsigned int pwm = ((PWM_MAX * (value * _frequency / 1000000.0)) + 0.5);
	pwmWrite(pin, pwm);
}

void PWM::high(unsigned int pin) const {
	digitalWrite(pin, true);
}

void PWM::low(unsigned int pin) const {
	digitalWrite(pin, false);
}

} /* namespace wanhive */
