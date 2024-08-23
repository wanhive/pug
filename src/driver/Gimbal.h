/*
 * Gimbal.h
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

#ifndef WH_DRIVER_GIMBAL_H_
#define WH_DRIVER_GIMBAL_H_
#include "Servo.h"

namespace wanhive {
/**
 * 3-axis servo gimbal controller.
 * @note Uses standard servo to fix the orientation with 1 degree accuracy
 * @note Default PCA9685 pin map: Pan(0), Roll(2), Tilt(4)
 */
class Gimbal: protected Servo {
public:
	/**
	 * Constructor: initializes the controller.
	 * @param bus i2c adapter's identifier
	 * @param address device identifier
	 */
	Gimbal(unsigned int bus, unsigned int address = PCA9685::I2C_ADDR);
	/**
	 * Constructor: initializes the controller.
	 * @param path i2c adapter's pathname
	 * @param address device identifier
	 */
	Gimbal(const char *path, unsigned int address = PCA9685::I2C_ADDR);
	/**
	 * Destructor: closes the i2c bus.
	 */
	~Gimbal();

	/**
	 * Controls the pan axis.
	 * @param value orientation in degrees
	 */
	void pan(unsigned int value);
	/**
	 * Controls the roll axis.
	 * @param value orientation in degrees
	 */
	void roll(unsigned int value);
	/**
	 * Controls the tilt axis.
	 * @param value orientation in degrees
	 */
	void tilt(unsigned int value);
	/**
	 * Centers (90 degrees) all the three axes.
	 */
	void center();
public:
	/*! Minimum pan angle in degrees */
	static constexpr unsigned int PAN_MIN = 0;
	/*! Maximum pan angle in degrees */
	static constexpr unsigned int PAN_MAX = 180;
	/*! Minimum roll angle in degrees */
	static constexpr unsigned int ROLL_MIN = 0;
	/*! Maximum roll angle in degrees */
	static constexpr unsigned int ROLL_MAX = 180;
	/*! Minimum tilt angle in degrees */
	static constexpr unsigned int TILT_MIN = 0;
	/*! Maximum tilt angle in degrees */
	static constexpr unsigned int TILT_MAX = 180;

	/*! Pan control pin */
	static constexpr unsigned int PAN_CTRL = 0;
	/*! Roll control pin */
	static constexpr unsigned int ROLL_CTRL = 2;
	/*! Tilt control pin */
	static constexpr unsigned int TILT_CTRL = 4;
private:
	struct {
		unsigned int pan { 360 };
		unsigned int roll { 360 };
		unsigned int tilt { 360 };
	} axis;
};

} /* namespace wanhive */

#endif /* WH_DRIVER_GIMBAL_H_ */
