/**
 * @file ServoControl.h
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

#ifndef WH_DEVICE_SERVOCONTROL_H_
#define WH_DEVICE_SERVOCONTROL_H_

/*! @namespace wanhive */
namespace wanhive {
/**
 * PWM control structure for servo motors.
 */
class ServoControl {
public:
	/**
	 * Default constructor: loads the default configuration.
	 */
	ServoControl() noexcept;
	/**
	 * Constructor: loads the user-provided configuration values.
	 * @param center neutral pulse width (microseconds)
	 * @param slope control signal's linear gradient
	 */
	ServoControl(unsigned int center, double slope) noexcept;
	/**
	 * Destructor
	 */
	~ServoControl();
	/**
	 * Calculates pulse width for a given position.
	 * @param angle servo's position in degrees
	 * @return pulse width in microseconds
	 */
	double pulse(double angle) const noexcept;
	/**
	 * Updates the configuration.
	 * @param center neutral pulse width (microseconds)
	 * @param slope control signal's linear gradient
	 */
	void get(unsigned int &center, double &slope) const noexcept;
	/**
	 * Returns the configuration.
	 * @param center neutral pulse width (microseconds)
	 * @param slope control signal's linear gradient
	 */
	void set(unsigned int center, double slope) noexcept;
	/**
	 * Returns the neutral pulse width value.
	 * @return neutral pulse width (microseconds)
	 */
	unsigned int getCenter() const noexcept;
	/**
	 * Updates the neutral pulse width value.
	 * @param center neutral pulse width (microseconds)
	 */
	void setCenter(unsigned int center) noexcept;
	/**
	 * Returns the linear gradient value.
	 * @return control signal's linear gradient
	 */
	double getSlope() const noexcept;
	/**
	 * Updates the linear gradient value.
	 * @param slope control signal's linear gradient
	 */
	void setSlope(double slope) noexcept;
	/**
	 * Loads default settings.
	 */
	void reset() noexcept;
public:
	/*! Default neutral pulse width (microseconds) */
	static constexpr unsigned int CENTER = 1500;
	/*! Default linear gradient (slope) */
	static constexpr double SLOPE = ((2000 - 1000) / 180.0);
private:
	unsigned int center;
	double slope;
};

} /* namespace wanhive */

#endif /* WH_DEVICE_SERVOCONTROL_H_ */
