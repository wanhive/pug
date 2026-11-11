/**
 * @file AirQuality.h
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

#ifndef WH_DEVICE_AIRQUALITY_H_
#define WH_DEVICE_AIRQUALITY_H_

/*! @namespace wanhive */
namespace wanhive {
/**
 * Air quality data
 */
struct AirQuality {
	/*! Ambient temperature (K) */
	double temperature;
	/*! Relative humidity (%RH) */
	double humidity;
	/*! Atmospheric pressure (Pascal) */
	double pressure;
	/*! Gas index */
	unsigned int gas;
};

} /* namespace wanhive */

#endif /* WH_DEVICE_AIRQUALITY_H_ */
