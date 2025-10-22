/*
 * GeoLocation.h
 *
 * Copyright (C) 2020 Wanhive Systems Private Limited (info@wanhive.com)
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

#ifndef WH_DEVICE_GEOLOCATION_H_
#define WH_DEVICE_GEOLOCATION_H_

namespace wanhive {
/**
 * Geo-location data structure.
 */
struct GeoLocation {
	/*! Satellite lock mode: [2D (2); 3D (3)] */
	unsigned int mode;
	/*! Unix time stamp */
	double timestamp;
	/*! Latitude in degrees */
	double latitude;
	/*! Longitude in degrees */
	double longitude;
	/*! Altitude over mean sea level (meter) */
	double altitude;
	/*! Speed (meter/second) */
	double speed;
	/*! Heading relative to true North */
	double heading;
	/*! Vertical speed (meter/second) */
	double climb;
};

}  // namespace wanhive

#endif /* WH_DEVICE_GEOLOCATION_H_ */
