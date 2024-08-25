/*
 * PWMController.h
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

#ifndef WH_DEVICE_PWMCONTROLLER_H_
#define WH_DEVICE_PWMCONTROLLER_H_
#include "../driver/PCA9685.h"

namespace wanhive {
/**
 * PWM controller with a fixed output modulation frequency.
 * @note Produces servo/digital/pwm outputs
 */
class PWMController: protected PCA9685 {
public:
	/**
	 * Constructor: initializes the controller and sets it's frequency.
	 * @param bus i2c adapter's identifier
	 * @param address device identifier
	 */
	PWMController(unsigned int bus, unsigned int address = I2C_ADDR);
	/**
	 * Constructor: initializes the controller and sets it's frequency.
	 * @param path adapter's pathname
	 * @param address device identifier
	 */
	PWMController(const char *path, unsigned int address = I2C_ADDR);
	/**
	 * Destructor: closes the i2c bus.
	 */
	~PWMController();
	/**
	 * Applies pulse of a given width to the servo motor's control wire.
	 * @param pin the pin number (0-15)
	 * @param pulse pulse width in milliseconds
	 */
	void servo(unsigned int pin, float pulse = SERVO_CENTER) const;
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
	/**
	 * Sets up PWM output (turn-on time and duty cycle).
	 * @param pin the pin number (0-15)
	 * @param delay turn-on time (%)
	 * @param duty PWM duty cycle (%)
	 */
	void pulse(unsigned int pin, unsigned int delay, unsigned int duty) const;
private:
	using PCA9685::getFrequency;
	using PCA9685::setFrequency;
public:
	/**
	 * Output modulation frequency (Hz)
	 */
	static const unsigned int FREQUENCY;
	/**
	 * Pulse width in milliseconds to center the servo motor
	 */
	static const float SERVO_CENTER;
};

} /* namespace wanhive */

#endif /* WH_DEVICE_PWMCONTROLLER_H_ */
