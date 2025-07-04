/*
 * ServoControl.cpp
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

#include "ServoControl.h"

namespace wanhive {

ServoControl::ServoControl() noexcept {
	reset();
}

ServoControl::ServoControl(unsigned int center, double slope) noexcept {
	set(center, slope);
}

ServoControl::~ServoControl() {

}

double ServoControl::pulse(double angle) const noexcept {
	return (angle * slope + center);
}

void ServoControl::get(unsigned int &center, double &slope) const noexcept {
	center = getCenter();
	slope = getSlope();
}

void ServoControl::set(unsigned int center, double slope) noexcept {
	setCenter(center);
	setSlope(slope);
}

unsigned int ServoControl::getCenter() const noexcept {
	return center;
}

void ServoControl::setCenter(unsigned int center) noexcept {
	this->center = center;
}

double ServoControl::getSlope() const noexcept {
	return slope;
}

void ServoControl::setSlope(double slope) noexcept {
	this->slope = slope;
}

void ServoControl::reset() noexcept {
	setCenter(CENTER);
	setSlope(SLOPE);
}

} /* namespace wanhive */
