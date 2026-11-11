/**
 * @file Motion.h
 *
 * Copyright (C) 2026 Wanhive Systems Private Limited (info@wanhive.com)
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

#ifndef WH_DEVICE_MOTION_H_
#define WH_DEVICE_MOTION_H_
#include "GeoLocation.h"
#include <wanhive/base/ds/Spatial.h>

/*! @namespace wanhive */
namespace wanhive {
/**
 * Enumeration of motion sensor types
 */
enum MotionType : unsigned char {
	MOTION_COMPASS = 0x01,/**< Compass */
	MOTION_IMU = 0x02, /**< IMU */
	MOTION_NAV = 0x04, /**< GNSS */
	MOTION_ALL = 0x07 /**< All */
};

/**
 * Motion data structure
 */
struct Motion {
	/*! Available features */
	unsigned char features;
	/*! Compass heading data */
	Spatial<double> compass;
	/*! Accelerometer data */
	Spatial<double> linear;
	/*! Gyroscope data */
	Spatial<double> angular;
	/*! GNSS data */
	GeoLocation track;
};

} /* namespace wanhive */

#endif /* WH_DEVICE_MOTION_H_ */
