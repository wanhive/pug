/*
 * PWM.h
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

#ifndef WH_DEVICE_PWM_H_
#define WH_DEVICE_PWM_H_
#include "../driver/PCA9685.h"

namespace wanhive {
/**
 * PWM controller with a fixed output modulation frequency.
 * @note Produces servo/digital/pwm outputs
 */
class PWM: protected PCA9685 {
public:
	/**
	 * Constructor: initializes the controller and sets it's frequency.
	 * @param frequency output modulation frequency (Hz)
	 * @param bus i2c adapter's identifier
	 * @param address  device identifier
	 */
	PWM(unsigned int frequency, unsigned int bus, unsigned int address =
			I2C_ADDR);
	/**
	 * Constructor: initializes the controller and sets it's frequency.
	 * @param frequency output modulation frequency (Hz)
	 * @param path i2c adapter's pathname
	 * @param address  device identifier
	 */
	PWM(unsigned int frequency, const char *path, unsigned int address =
			I2C_ADDR);
	/**
	 * Destructor: closes the i2c bus.
	 */
	~PWM();
	/**
	 * Applies pulse of a given width to the servo motor's control wire.
	 * @param pin the pin number (0-15)
	 * @param value pulse width in microseconds
	 */
	void servo(unsigned int pin, unsigned int value) const;
	/**
	 * Sets a pin to logic high.
	 * @param pin the pin number (0-15)
	 */
	void high(unsigned int pin) const;
	/**
	 * Sets a pin to logic low.
	 * @param pin the pin number (0-15)
	 */
	void low(unsigned int pin) const;
private:
	using PCA9685::setFrequency;
public:
	/*! Default output modulation frequency (Hz) */
	static constexpr unsigned int FREQUENCY = 50;
private:
	const unsigned int _frequency;
};

} /* namespace wanhive */

#endif /* WH_DEVICE_PWM_H_ */
