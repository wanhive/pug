/*
 * DataLogger.h
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

#ifndef WH_DEVICE_DATALOGGER_H_
#define WH_DEVICE_DATALOGGER_H_
#include <wanhive/base/unix/File.h>

namespace wanhive {
/**
 * Bare minimum data logger.
 * @note Records sensor data over time.
 */
class DataLogger: protected File {
public:
	/**
	 * Constructor: initializes the data logger.
	 * @param path log file's absolute pathname
	 * @param events maximum number of events
	 */
	DataLogger(const char *path, unsigned long long events);
	/**
	 * Constructor: initializes the data logger.
	 * @param fd log file's descriptor
	 * @param events maximum number of events
	 */
	DataLogger(int fd, unsigned long long events);
	/**
	 * Destructor: closes the log file.
	 */
	~DataLogger();
	/**
	 * Writes new data into the log file.
	 * @param format data format string
	 */
	void insert(const char *format, ...);
	/**
	 * Truncates the log file.
	 * @param forced true to force, false to truncate only on overflow
	 */
	void reset(bool forced = false);
private:
	const unsigned long long limit;
	unsigned long long events { 0 };
};

} /* namespace wanhive */

#endif /* WH_DEVICE_DATALOGGER_H_ */
