/*
 * GPS.cpp
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

#include "GPS.h"
#include <cmath>
#include <cstring>

#if GPSD_API_MAJOR_VERSION < 10
#error "Incompatible GPSD API version."
#endif

namespace {

const char *hosts[][2] = { { "localhost", DEFAULT_GPSD_PORT }, {
GPSD_SHARED_MEMORY, nullptr } };

}  // namespace

namespace wanhive {

GPS::GPS(GPSHost host) noexcept :
		host { host }, connected { false } {

}

GPS::~GPS() {
	reset();
}

bool GPS::read(GeoLocation &location) noexcept {
	if (!isConnected() && !connect()) {
		return false;
	}

	gps_clear_fix(&data.fix);
	auto nRead = gps_read(&data, nullptr, 0);
	if (nRead < 0) {
		reset();
		return false;
	} else if (!nRead || !hasData()) {
		return false;
	} else {
		getData(location);
		return true;
	}
}

void GPS::reset() noexcept {
	disconnect();
}

void GPS::getData(GeoLocation &location) const noexcept {
	memset(&location, 0, sizeof(location));
	const auto &fix = data.fix;
	location.timestamp = fix.time.tv_sec + (fix.time.tv_nsec / 1000000000.0);
	if (std::isfinite(fix.latitude)) {
		location.latitude = fix.latitude;
	}
	if (std::isfinite(fix.longitude)) {
		location.longitude = fix.longitude;
	}
	if (std::isfinite(fix.altMSL)) {
		location.altitude = fix.altMSL;
	}
	if (std::isfinite(fix.speed)) {
		location.speed = fix.speed;
	}
	if (std::isfinite(fix.track)) {
		location.heading = fix.track;
	}
	if (std::isfinite(fix.climb)) {
		location.climb = fix.climb;
	}
	location.mode = fix.mode;
}

bool GPS::hasData() noexcept {
	return isConnected() && (data.set)
			&& (data.fix.mode == MODE_2D || data.fix.mode == MODE_3D);
}

bool GPS::isConnected() noexcept {
	return connected;
}

bool GPS::connect() noexcept {
	disconnect();
	auto addr = hosts[host];
	if (gps_open(addr[0], addr[1], &data) != 0) {
		return false;
	} else {
		gps_stream(&data, WATCH_ENABLE, nullptr);
		connected = true;
		return true;
	}
}

void GPS::disconnect() noexcept {
	if (connected) {
		gps_stream(&data, WATCH_DISABLE, nullptr);
		gps_close(&data);
		connected = false;
	}
}

} /* namespace wanhive */
