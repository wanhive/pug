/*
 * PCA9685.h
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

#ifndef WH_DRIVER_PCA9685_H_
#define WH_DRIVER_PCA9685_H_
#include "../physical/SMBus.h"

namespace wanhive {
/**
 * User space PCA9685 driver.
 * @note PCA9685 is a 16-channel, 12-bit resolution PWM controller.
 * @ref http://www.nxp.com/documents/data_sheet/PCA9685.pdf
 */
class PCA9685: protected SMBus {
public:
	/**
	 * Constructor: initializes a PCA9685 driver.
	 * @param bus i2c adapter's identifier
	 * @param address device identifier (typically 0x40)
	 */
	PCA9685(unsigned int bus, unsigned int address = I2C_ADDR);
	/**
	 * Constructor: initializes a PCA9685 driver.
	 * @param path i2c adapter's pathname
	 * @param address device identifier (typically 0x40)
	 */
	PCA9685(const char *path, unsigned int address = I2C_ADDR);
	/**
	 * Destructor: closes the i2c bus.
	 */
	~PCA9685() noexcept;
	/**
	 * Simple PWM control which sets the on-tick to 0 and the off-tick to a
	 * given value.
	 * @param pin the pin number (0-15), ALL_PIN to select all.
	 * @param value the value to set. If the value is 0 then full-off will be
	 * enabled. If the value is >= PWM_MAX then full-on will be enabled. Every
	 * value in between enables the PWM output.
	 */
	void pwmWrite(unsigned int pin, unsigned int value) const;
	/**
	 * Simple full-on and full-off control.
	 * @param pin the pin number (0-15), ALL_PIN to select all.
	 * @param value if value is false, full-off will be enabled. If value is
	 * true, full-on will be enabled.
	 */
	void digitalWrite(unsigned int pin, bool value) const;
	/**
	 * Reads the current output frequency (with internal oscillator).
	 * @return output frequency in Hz
	 */
	unsigned int getFrequency() const;
	/**
	 * Sets the output frequency (with internal oscillator).
	 * @param frequency desired output frequency
	 * @return the frequency been set
	 */
	unsigned int setFrequency(unsigned int frequency) const;
	/**
	 * Restarts the device.
	 */
	void restart() const;
	/**
	 * Puts the device into sleep mode.
	 */
	void sleep() const;
	/**
	 * Wakes up the device from sleep mode.
	 */
	void wakeUp() const;
	/**
	 * Writes the on and off ticks to a pin (deactivates full-on and full-off).
	 * @param pin the pin number (0-15), ALL_PIN to select all.
	 * @param on the on tick
	 * @param off the off tick
	 */
	void write(unsigned int pin, unsigned short on, unsigned short off) const;
	/**
	 * Reads both on and off registers as 16 bit of data. To get PWM mask each
	 * value with 0xFFF. To get full-on or full-off bit mask with 0x1000.
	 * @param pin the pin number (0-15), ALL_PIN to select all.
	 * @param on the on register value
	 * @param off the off register value
	 */
	void read(unsigned int pin, unsigned short &on, unsigned short &off) const;
	/**
	 * Activates or deactivates full-on.
	 * @param pin the pin number (0-15), ALL_PIN to select all.
	 * @param flag true for full-on, false for PWM
	 */
	void fullOn(unsigned int pin, bool flag) const;
	/**
	 * Activates or deactivates full-off.
	 * @param pin the pin number (0-15), ALL_PIN to select all.
	 * @param flag true for full-off, false for PWM or full-on
	 */
	void fullOff(unsigned int pin, bool flag) const;
	/**
	 * Sets up the output mode.
	 * @param invert true to invert the output logic state, false otherwise
	 * @param openDrain true for open drain, false for totem-pole
	 */
	void setOutputMode(bool invert, bool openDrain) const;
private:
	void setup() const;
public:
	/**
	 * Default I2C address
	 */
	static constexpr unsigned char I2C_ADDR = 0x40;
	/**
	 * The maximum PWM value (exclusive).
	 */
	static constexpr unsigned int PWM_MAX = 4096;
	/**
	 * All pins selector.
	 */
	static constexpr unsigned int ALL_PIN = 16;
	/**
	 * Minimum allowed output frequency (Hz).
	 */
	static constexpr unsigned int MIN_FREQUENCY = 40;
	/**
	 * Maximum allowed output frequency (Hz).
	 */
	static constexpr unsigned int MAX_FREQUENCY = 1000;
	/**
	 * Internal oscillator's frequency.
	 */
	static constexpr unsigned int OSC_CLOCK = 25000000;
};

} /* namespace wanhive */

#endif /* WH_DRIVER_PCA9685_H_ */
