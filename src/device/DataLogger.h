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
 * Basic data logger.
 */
class DataLogger: protected File {
public:
	/**
	 * Constructor: initializes the data logger.
	 * @param path log file's absolute pathname
	 */
	DataLogger(const char *path);
	/**
	 * Constructor: initializes the data logger.
	 * @param fd log file's descriptor
	 */
	DataLogger(int fd);
	/**
	 * Destructor: closes the log file.
	 */
	~DataLogger();
	/**
	 * Writes data to the log file.
	 * @param format format string, subsequent arguments are converted
	 * for output (just like printf)
	 */
	void insert(const char *format, ...);
	/**
	 * Truncates the log file.
	 */
	void reset();
};

} /* namespace wanhive */

#endif /* WH_DEVICE_DATALOGGER_H_ */
