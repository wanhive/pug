/*
 * GPS.h
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

#ifndef WH_DEVICE_GPS_H_
#define WH_DEVICE_GPS_H_
#include <gps.h>

namespace wanhive {
/**
 * Geolocation data.
 */
struct GeoLocation {
	/*! Satellite lock mode: [2D (2); 3D (3)] */
	unsigned int mode;
	/*! Unix timestamp */
	double timestamp;
	/*! Latitude in degrees */
	double latitude; //Latitude
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

/**
 * GPS service daemon connection options.
 */
enum GPSHost {
	WH_GPS_SOCK,/**< Socket connection */
	WH_GPS_SHM /**< Shared memory */
};

/**
 * GPS tracker.
 * @note Relies on the GPSD service.
 * @ref https://gpsd.gitlab.io/gpsd/
 */
class GPS {
public:
	/**
	 * Constructor: initializes the object.
	 */
	GPS(GPSHost host = WH_GPS_SOCK) noexcept;
	/**
	 * Destructor: disconnects from the GPS daemon.
	 */
	~GPS();
	/**
	 * Reads geo-location data.
	 * @param location output data
	 * @return true on success, false on error
	 */
	bool read(GeoLocation &location) noexcept;
	/**
	 * Disconnects from the GPS daemon.
	 */
	void reset() noexcept;
private:
	void getData(GeoLocation &location) const noexcept;
	bool hasData() noexcept;
	bool isConnected() noexcept;
	bool connect() noexcept;
	void disconnect() noexcept;
private:
	GPSHost host;
	bool connected;
	gps_data_t data;
};

} /* namespace wanhive */

#endif /* WH_DEVICE_GPS_H_ */
