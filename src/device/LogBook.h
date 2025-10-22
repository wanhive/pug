/*
 * LogBook.h
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

#ifndef WH_DEVICE_LOGBOOK_H_
#define WH_DEVICE_LOGBOOK_H_
#include <wanhive/base/unix/File.h>
#include <cstdarg>

namespace wanhive {
/**
 * Record of sensor data.
 */
class LogBook: protected File {
public:
	/**
	 * Constructor: initializes the record.
	 * @param path file's absolute pathname
	 */
	LogBook(const char *path);
	/**
	 * Constructor: initializes the record.
	 * @param fd file's descriptor
	 */
	LogBook(int fd);
	/**
	 * Destructor: closes the record.
	 */
	~LogBook();
	/**
	 * Adds new information to the record.
	 * @param format the format string, subsequent arguments are converted
	 * for output (just like printf)
	 */
	void enter(const char *format, ...) const;
	/**
	 * Adds new information to the record.
	 * @param format the format string
	 * @param ap arguments for output
	 */
	void enter(const char *format, va_list ap) const;
	/**
	 * Removes all data from the record.
	 */
	void clear() const;
};

} /* namespace wanhive */

#endif /* WH_DEVICE_LOGBOOK_H_ */
