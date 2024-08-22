/*
 * PCA9685.cpp
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

#include "PCA9685.h"
#include <wanhive/base/ds/Twiddler.h>
#include <wanhive/base/Timer.h>

namespace {

constexpr unsigned char MODE1_REG = 0x0;
constexpr unsigned char MODE2_REG = 0x01;

constexpr unsigned char RESTART_MASK = 0x80;
constexpr unsigned char SLEEP_MASK = 0x10;
constexpr unsigned char AI_MASK = 0x20;

constexpr unsigned char INVRT_MASK = 0x10;

constexpr unsigned char FULL_MASK = 0x10;

/**
 * Locates the correct LEDX_ON_L register for the pin number starting at 0.
 */
unsigned int baseRegister(unsigned int pin) {
	constexpr unsigned int PIN_COUNT = 16; //Total number of pins
	constexpr unsigned int LED0_ON_L = 0x6; //First LED
	constexpr unsigned int ALL_LED_ON_L = 0xFA; //All LED
	if (pin < PIN_COUNT) {
		return LED0_ON_L + (pin * 4);
	} else {
		return ALL_LED_ON_L;
	}
}

}  // namespace

namespace wanhive {

PCA9685::PCA9685(unsigned int bus, unsigned int address) :
		SMBus(bus, address) {
	setup();
}

PCA9685::PCA9685(const char *path, unsigned int address) :
		SMBus(path, address) {
	setup();
}

PCA9685::~PCA9685() noexcept {

}

void PCA9685::pwmWrite(unsigned int pin, unsigned int value) const {
	if (value >= PWM_MAX) {
		fullOn(pin, true);
	} else if (value > 0) {
		write(pin, 0, value);
	} else {
		fullOff(pin, true);
	}
}

void PCA9685::digitalWrite(unsigned int pin, bool value) const {
	if (value) {
		fullOn(pin, true);
	} else {
		fullOff(pin, true);
	}
}

unsigned int PCA9685::setFrequency(unsigned int frequency) const {
	/*
	 * Using the internal oscillator
	 * Limit the frequency to the range [MIN_FREQUENCY, MAX_FREQUENCY]
	 * The data sheet says 24Hz to 1526Hz
	 */
	constexpr double OSCILLATOR_FREQUENCY = 25000000;
	constexpr unsigned char PRESCALE_MIN = 0x03;
	constexpr unsigned char PRESCALE_REG = 0xFE;
	frequency = (
			frequency < MIN_FREQUENCY ?
					MIN_FREQUENCY :
					(frequency > MAX_FREQUENCY ? MAX_FREQUENCY : frequency));
	unsigned char prescale = (unsigned char) (((OSCILLATOR_FREQUENCY
			/ (frequency * 4096)) + 0.5) - 1);

	if (prescale < PRESCALE_MIN) { //Just in case
		prescale = PRESCALE_MIN;
	}

	unsigned char state;
	SMBus::read(MODE1_REG, state);
	//Clear the restart bit
	state &= ~RESTART_MASK;
	//Go to sleep (set the sleep bit)
	state |= SLEEP_MASK;
	SMBus::write(MODE1_REG, state);
	//Set prescale
	SMBus::write(PRESCALE_REG, prescale);
	//Wake up (clear the sleep bit)
	state &= ~SLEEP_MASK;
	SMBus::write(MODE1_REG, state);
	//Allow the oscillator to stabilize
	Timer::sleep(1);
	//Restart PWM
	state |= (RESTART_MASK | AI_MASK);
	SMBus::write(MODE1_REG, state);

	return frequency;
}

void PCA9685::restart() const {
	unsigned char state;
	SMBus::read(MODE1_REG, state);
	if (state & RESTART_MASK) {
		state &= ~RESTART_MASK;
		state &= ~SLEEP_MASK;
		SMBus::write(MODE1_REG, state);
		Timer::sleep(1);
	}

	state |= RESTART_MASK;
	SMBus::write(MODE1_REG, state);
}

void PCA9685::sleep() const {
	unsigned char state;
	SMBus::read(MODE1_REG, state);
	state |= SLEEP_MASK;
	SMBus::write(MODE1_REG, state);
}

void PCA9685::wakeUp() const {
	unsigned char state;
	SMBus::read(MODE1_REG, state);
	state &= ~SLEEP_MASK;
	SMBus::write(MODE1_REG, state);
	Timer::sleep(1);
}

void PCA9685::write(unsigned int pin, unsigned short on,
		unsigned short off) const {
	//Use only the lowest 12 bits, clear the full-on and full-off bits.
	on &= 0x0FFF;
	off &= 0x0FFF;

	//Write to on and off registers
	auto reg = baseRegister(pin);
	SMBus::write(reg, on);      //LEDX_ON
	SMBus::write(reg + 2, off); //LEDX_OFF
}

void PCA9685::read(unsigned int pin, unsigned short &on,
		unsigned short &off) const {
	auto reg = baseRegister(pin);
	SMBus::read(reg, on);
	SMBus::read(reg + 2, off);
}

void PCA9685::fullOn(unsigned int pin, bool flag) const {
	auto reg = baseRegister(pin) + 1; //LEDX_ON_H
	unsigned char state;
	SMBus::read(reg, state);
	state = Twiddler::mask(state, FULL_MASK, flag);
	SMBus::write(reg, state);

	//Because full-off takes precedence
	if (flag) {
		fullOff(pin, false);
	}
}

void PCA9685::fullOff(unsigned int pin, bool flag) const {
	auto reg = baseRegister(pin) + 3; //LEDX_OFF_H
	unsigned char state;
	SMBus::read(reg, state);
	state = Twiddler::mask(state, FULL_MASK, flag);
	SMBus::write(reg, state);
}

void PCA9685::setOutputMode(bool invert, bool openDrain) const {
	constexpr unsigned char TOTEMPOLE_MASK = 0x04;
	unsigned char state;
	SMBus::read(MODE2_REG, state);

	if (invert) {
		state |= INVRT_MASK;
	} else {
		state &= ~INVRT_MASK;
	}

	if (openDrain) {
		state &= ~TOTEMPOLE_MASK;
	} else {
		state |= TOTEMPOLE_MASK;
	}

	SMBus::write(MODE2_REG, state);
}

void PCA9685::setup() const {
	unsigned char state;
	SMBus::read(MODE1_REG, state);
	//Enable register auto-increment
	state = (state & ~RESTART_MASK) | AI_MASK;
	SMBus::write(MODE1_REG, state);
}

} /* namespace wanhive */
