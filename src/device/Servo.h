/*
 * Servo.h
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

#ifndef WH_DEVICE_SERVO_H_
#define WH_DEVICE_SERVO_H_
#include "../driver/PCA9685.h"

namespace wanhive {
/**
 * Servo motor controller
 */
class Servo: protected PCA9685 {
public:
	/**
	 * Constructor: initializes the driver and sets it's refresh rate.
	 * @param bus i2c adapter's identifier
	 * @param address device identifier
	 */
	Servo(unsigned int bus, unsigned int address = PCA9685::I2C_ADDR);
	/**
	 * Constructor: initializes the driver and sets it's refresh rate.
	 * @param path i2c adapter's pathname
	 * @param address device identifier
	 */
	Servo(const char *path, unsigned int address = PCA9685::I2C_ADDR);
	/**
	 * Destructor: closes the i2c bus.
	 */
	~Servo();
	/**
	 * Applies pulse of a given width to the servo motor's control wire.
	 * @param pin the pin number (0-15)
	 * @param width pulse width in milliseconds
	 */
	void pulse(unsigned int pin, float width = NEUTRAL_PULSE) const;
public:
	/**
	 * Default refresh rate (Hz)
	 */
	static const unsigned int REFRESH_RATE;
	/**
	 * Common pulse width in milliseconds for the central/neutral position
	 */
	static const float NEUTRAL_PULSE;
};

} /* namespace wanhive */

#endif /* WH_DEVICE_SERVO_H_ */
