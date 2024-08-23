/*
 * Servo.cpp
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

#include "Servo.h"
#ifndef WH_SERVO_REFRESH_RATE
#define WH_SERVO_REFRESH_RATE 50U
#endif
#ifndef WH_SERVO_NEUTRAL_PULSE
#define WH_SERVO_NEUTRAL_PULSE 1.5f
#endif

namespace wanhive {

const unsigned int Servo::REFRESH_RATE = WH_SERVO_REFRESH_RATE;
const float Servo::NEUTRAL_PULSE = WH_SERVO_NEUTRAL_PULSE;

Servo::Servo(unsigned int bus, unsigned int address) :
		PCA9685(bus, address) {
	setFrequency(REFRESH_RATE);
}

Servo::Servo(const char *path, unsigned int address) :
		PCA9685(path, address) {
	setFrequency(REFRESH_RATE);
}

Servo::~Servo() {

}

void Servo::pulse(unsigned int pin, float width) const {
	auto period = 1000.0f / REFRESH_RATE;
	int value = (PWM_MAX * width / period + 0.5f);
	value = (value >= 0) ? value : 0;
	pwmWrite(pin, value);
}

} /* namespace wanhive */
