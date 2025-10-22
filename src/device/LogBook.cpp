/*
 * LogBook.cpp
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

#include "LogBook.h"
#include <wanhive/base/Storage.h>
#include <wanhive/base/unix/SystemException.h>

namespace wanhive {

LogBook::LogBook(const char *path) :
		File { path, (O_WRONLY | O_CREAT | O_APPEND), (S_IRWXU | S_IRGRP
				| S_IROTH) } {
}

LogBook::LogBook(int fd) :
		File { fd } {

}

LogBook::~LogBook() {

}

void LogBook::enter(const char *format, ...) const {
	va_list ap;
	va_start(ap, format);
	try {
		enter(format, ap);
		va_end(ap);
	} catch (const BaseException &e) {
		va_end(ap);
		throw;
	}
}

void LogBook::enter(const char *format, va_list ap) const {
	auto status = vdprintf(File::get(), format, ap);
	if (status < 0) {
		throw SystemException();
	}
}

void LogBook::clear() const {
	Storage::truncate(File::get(), 0);
	Storage::seek(File::get(), 0, SEEK_SET);
}

} /* namespace wanhive */
