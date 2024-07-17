/*
 * GPIO.cpp
 *
 * Copyright (C) 2023 Amit Kumar (amitkriit@gmail.com)
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

#include "GPIO.h"
#include <wanhive/base/unix/SystemException.h>
#include <wanhive/base/unix/FileSystem.h>
#include <wanhive/base/unix/FStat.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <climits>
#include <cstdio>
#include <unistd.h>

namespace {

const char* pathName(unsigned id, char *buffer, unsigned size) {
	auto ret = snprintf(buffer, size, "/dev/gpiochip%u", id);
	if (ret > 0 && ((unsigned) ret < size)) {
		return buffer;
	} else {
		throw wanhive::Exception(wanhive::EX_OVERFLOW);
	}
}

const char* directionString(wanhive::LineDirection direction) noexcept {
	switch (direction) {
	case wanhive::LineDirection::NONE:
		return "DISABLED";
	case wanhive::LineDirection::INPUT:
		return "INPUT";
	case wanhive::LineDirection::OUTPUT:
		return "OUTPUT";
	case wanhive::LineDirection::BOTH:
		return "BOTH";
	default:
		return "";
	}
}

uint64_t getDirection(wanhive::LineDirection direction) noexcept {
	switch (direction) {
	case wanhive::LineDirection::NONE:
		return 0;
	case wanhive::LineDirection::INPUT:
		return GPIO_V2_LINE_FLAG_INPUT;
	case wanhive::LineDirection::OUTPUT:
		return GPIO_V2_LINE_FLAG_OUTPUT;
	case wanhive::LineDirection::BOTH:
		return (GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_OUTPUT);
	default:
		return 0;
	}
}

wanhive::LineDirection getDirection(uint64_t flags) noexcept {
	if ((flags & GPIO_V2_LINE_FLAG_INPUT)
			&& (flags & GPIO_V2_LINE_FLAG_OUTPUT)) {
		return wanhive::LineDirection::BOTH;
	} else if (flags & GPIO_V2_LINE_FLAG_INPUT) {
		return wanhive::LineDirection::INPUT;
	} else if (flags & GPIO_V2_LINE_FLAG_OUTPUT) {
		return wanhive::LineDirection::OUTPUT;
	} else {
		return wanhive::LineDirection::NONE;
	}
}

const char* stateString(wanhive::LineState state) {
	switch (state) {
	case wanhive::LineState::ACTIVE_HIGH:
		return "ACTIVE-HIGH";
	case wanhive::LineState::ACTIVE_LOW:
		return "ACTIVE-LOW";
	default:
		return "";
	}
}

uint64_t getState(wanhive::LineState state) noexcept {
	switch (state) {
	case wanhive::LineState::ACTIVE_HIGH:
		return 0;
	case wanhive::LineState::ACTIVE_LOW:
		return GPIO_V2_LINE_FLAG_ACTIVE_LOW;
	default:
		return 0;
	}
}

wanhive::LineState getState(uint64_t flags) noexcept {
	if (flags & GPIO_V2_LINE_FLAG_ACTIVE_LOW) {
		return wanhive::LineState::ACTIVE_LOW;
	} else {
		return wanhive::LineState::ACTIVE_HIGH;
	}
}

const char* driveString(wanhive::LineDrive drive) noexcept {
	switch (drive) {
	case wanhive::LineDrive::NONE:
		return "DEFAULT";
	case wanhive::LineDrive::OPEN_DRAIN:
		return "OPEN-DRAIN";
	case wanhive::LineDrive::OPEN_SOURCE:
		return "OPEN-SOURCE";
	default:
		return "";
	}
}

uint64_t getDrive(wanhive::LineDrive drive) noexcept {
	switch (drive) {
	case wanhive::LineDrive::NONE:
		return 0;
	case wanhive::LineDrive::OPEN_DRAIN:
		return GPIO_V2_LINE_FLAG_OPEN_DRAIN;
	case wanhive::LineDrive::OPEN_SOURCE:
		return GPIO_V2_LINE_FLAG_OPEN_SOURCE;
	default:
		return 0;
	}
}

wanhive::LineDrive getDrive(uint64_t flags) noexcept {
	if (flags & GPIO_V2_LINE_FLAG_OPEN_DRAIN) {
		return wanhive::LineDrive::OPEN_DRAIN;
	} else if (flags & GPIO_V2_LINE_FLAG_OPEN_SOURCE) {
		return wanhive::LineDrive::OPEN_SOURCE;
	} else {
		return wanhive::LineDrive::NONE;
	}
}

const char* biasString(wanhive::LineBias bias) noexcept {
	switch (bias) {
	case wanhive::LineBias::NONE:
		return "DEFAULT";
	case wanhive::LineBias::PULL_UP:
		return "PULL-UP";
	case wanhive::LineBias::PULL_DOWN:
		return "PULL-DOWN";
	case wanhive::LineBias::DISABLED:
		return "DISABLED";
	default:
		return "";
	}
}

uint64_t getBias(wanhive::LineBias bias) noexcept {
	switch (bias) {
	case wanhive::LineBias::NONE:
		return 0;
	case wanhive::LineBias::PULL_UP:
		return GPIO_V2_LINE_FLAG_BIAS_PULL_UP;
	case wanhive::LineBias::PULL_DOWN:
		return GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN;
	case wanhive::LineBias::DISABLED:
		return GPIO_V2_LINE_FLAG_BIAS_DISABLED;
	default:
		return 0;
	}
}

wanhive::LineBias getBias(uint64_t flags) noexcept {
	if (flags & GPIO_V2_LINE_FLAG_BIAS_PULL_UP) {
		return wanhive::LineBias::PULL_UP;
	} else if (flags & GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN) {
		return wanhive::LineBias::PULL_DOWN;
	} else if (flags & GPIO_V2_LINE_FLAG_BIAS_DISABLED) {
		return wanhive::LineBias::DISABLED;
	} else {
		return wanhive::LineBias::NONE;
	}
}

const char* eventString(const wanhive::LineEvent event) noexcept {
	switch (event) {
	case wanhive::LineEvent::NONE:
		return "NONE";
	case wanhive::LineEvent::RISING:
		return "RISING";
	case wanhive::LineEvent::FALLING:
		return "FALLING";
	case wanhive::LineEvent::BOTH:
		return "BOTH";
	default:
		return "";
	}
}

uint64_t getEvent(wanhive::LineEvent event) noexcept {
	switch (event) {
	case wanhive::LineEvent::NONE:
		return 0;
	case wanhive::LineEvent::RISING:
		return GPIO_V2_LINE_FLAG_EDGE_RISING;
	case wanhive::LineEvent::FALLING:
		return GPIO_V2_LINE_FLAG_EDGE_FALLING;
	case wanhive::LineEvent::BOTH:
		return (GPIO_V2_LINE_FLAG_EDGE_RISING | GPIO_V2_LINE_FLAG_EDGE_FALLING);
	default:
		return 0;
	}
}

wanhive::LineEvent getEvent(uint64_t flags) noexcept {
	if ((flags & GPIO_V2_LINE_FLAG_EDGE_RISING)
			&& (flags & GPIO_V2_LINE_FLAG_EDGE_FALLING)) {
		return wanhive::LineEvent::BOTH;
	} else if (flags & GPIO_V2_LINE_FLAG_EDGE_RISING) {
		return wanhive::LineEvent::RISING;
	} else if (flags & GPIO_V2_LINE_FLAG_EDGE_FALLING) {
		return wanhive::LineEvent::FALLING;
	} else {
		return wanhive::LineEvent::NONE;
	}
}

const char* clockString(const wanhive::LineClock ts) noexcept {
	switch (ts) {
	case wanhive::LineClock::NONE:
		return "DEFAULT";
	case wanhive::LineClock::REALTIME:
		return "REALTIME";
	default:
		return "";
	}
}

uint64_t getClock(wanhive::LineClock clock) noexcept {
	switch (clock) {
	case wanhive::LineClock::NONE:
		return 0;
	case wanhive::LineClock::REALTIME:
		return GPIO_V2_LINE_FLAG_EVENT_CLOCK_REALTIME;
	default:
		return 0;
	}
}

wanhive::LineClock getClock(uint64_t flags) noexcept {
	if (flags & GPIO_V2_LINE_FLAG_EVENT_CLOCK_REALTIME) {
		return wanhive::LineClock::REALTIME;
	} else {
		return wanhive::LineClock::NONE;
	}
}

uint64_t getFlags(const wanhive::LineConfig &lc) noexcept {
	uint64_t flags = 0;
	flags |= getDirection(lc.direction);
	flags |= getState(lc.state);
	flags |= getDrive(lc.drive);
	flags |= getBias(lc.bias);
	flags |= getEvent(lc.event);
	flags |= getClock(lc.clock);
	return flags;
}

void getConfiguration(uint64_t flags, wanhive::LineConfig &lc) noexcept {
	lc.direction = getDirection(flags);
	lc.state = getState(flags);
	lc.drive = getDrive(flags);
	lc.bias = getBias(flags);
	lc.event = getEvent(flags);
	lc.clock = getClock(flags);
}

void printConfiguration(FILE *stream, const wanhive::LineConfig &lc) noexcept {
	fprintf(stream, "Direction: %s, State: %s\n", directionString(lc.direction),
			stateString(lc.state));
	fprintf(stream, "Drive: %s, Bias: %s\n", driveString(lc.drive),
			biasString(lc.bias));
	fprintf(stream, "Events: %s, Clock: %s\n", eventString(lc.event),
			clockString(lc.clock));
	if (lc.debounce.on) {
		fprintf(stream, "Debounce-period (ms): %u\n", lc.debounce.period);
	}
}

}  // namespace

namespace wanhive {

GPIO::GPIO() noexcept {

}

GPIO::GPIO(const char *path) {
	open(path);
}

GPIO::GPIO(unsigned int id) {
	open(id);
}

GPIO::~GPIO() {
	close();
}

void GPIO::open(const char *path) {
	try {
		chip.open(path, O_RDONLY);
		FStat stat(chip.get());
		if (!stat.isCharSpecialFile()) {
			throw Exception(EX_OPERATION);
		}

		char path[PATH_MAX];
		char resolved[PATH_MAX];
		snprintf(path, sizeof(path), "/sys/dev/char/%u:%u/subsystem",
				stat.majorDeviceId(), stat.minorDeviceId());
		FileSystem::realPath(path, resolved);
		if (::strcmp("/sys/bus/gpio", resolved) != 0) {
			throw Exception(EX_OPERATION);
		}

		resetLines(readLinesCount());
	} catch (const BaseException &e) {
		close();
		throw;
	}
}

void GPIO::open(unsigned int id) {
	char buf[64];
	open(pathName(id, buf, sizeof(buf)));
}

void GPIO::close() noexcept {
	resetLines(0);
	chip.close();
}

unsigned int GPIO::getLinesCount() const noexcept {
	return lines.getLimit();
}

void GPIO::getLineConfiguration(unsigned int offset, LineConfig &config) const {
	gpio_v2_line_info info;
	::memset(&info, 0, sizeof(info));
	info.offset = offset;

	if (::ioctl(chip.get(), GPIO_V2_GET_LINEINFO_IOCTL, &info) == -1) {
		throw SystemException();
	}

	config.busy = (info.flags & GPIO_V2_LINE_FLAG_USED);
	getConfiguration(info.flags, config);
	for (unsigned int n = 0; n < info.num_attrs; ++n) {
		const auto &attr = info.attrs[n];
		if (attr.id == GPIO_V2_LINE_ATTR_ID_DEBOUNCE) {
			config.debounce.on = true;
			config.debounce.period = attr.debounce_period_us;
		} else if (attr.id == GPIO_V2_LINE_ATTR_ID_FLAGS) {
			getConfiguration(attr.flags, config);
		} else {
			//nothing
		}
	}
}

int GPIO::openLine(unsigned int offset, const LineConfig &config) {
	if (getLine(offset) > 0) {
		throw Exception(EX_OPERATION);
	}

	gpio_v2_line_request req;
	memset(&req, 0, sizeof(req));
	snprintf(req.consumer, sizeof(req.consumer) - 1, "wanhive");
	req.offsets[0] = offset;
	req.num_lines = 1;
	req.config.flags = getFlags(config);

	if (config.debounce.on) {
		req.config.num_attrs = 1;
		req.config.attrs[0].mask = 1;
		req.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_DEBOUNCE;
		req.config.attrs[0].attr.debounce_period_us = config.debounce.period;
	}

	if (::ioctl(chip.get(), GPIO_V2_GET_LINE_IOCTL, &req) == -1) {
		throw SystemException();
	}

	if (req.fd <= 0) {
		throw Exception(EX_OPERATION);
	}

	if (!setLine(req.fd, offset)) {
		::close(req.fd);
		throw Exception(EX_OPERATION);
	}

	return req.fd;
}

void GPIO::configureLine(unsigned int offset, const LineConfig &config) const {
	auto fd = getLine(offset);
	if (fd <= 0) {
		throw Exception(EX_RESOURCE);
	}
	gpio_v2_line_config req;
	::memset(&req, 0, sizeof(req));
	req.flags = getFlags(config);
	if (config.debounce.on) {
		req.num_attrs = 1;
		req.attrs[0].mask = 1;
		req.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_DEBOUNCE;
		req.attrs[0].attr.debounce_period_us = config.debounce.period;
	}

	if (::ioctl(fd, GPIO_V2_LINE_SET_CONFIG_IOCTL, &req) == -1) {
		throw SystemException();
	}
}

void GPIO::closeLine(unsigned int offset) noexcept {
	auto fd = getLine(offset);
	if (fd > 0) {
		::close(fd);
	}

	lines.put(-1, offset);
}

void GPIO::closeLines() noexcept {
	for (unsigned int i = 0; i < lines.getLimit(); ++i) {
		closeLine(i);
	}
}

int GPIO::getLine(unsigned int offset) const noexcept {
	int fd = -1;
	lines.get(fd, offset);
	return fd;
}

bool GPIO::read(unsigned int offset) const {
	gpio_v2_line_values data;
	memset(&data, 0, sizeof(data));
	data.mask = 1;
	if (ioctl(getLine(offset), GPIO_V2_LINE_GET_VALUES_IOCTL, &data) == -1) {
		throw SystemException();
	}
	return (data.bits & data.mask);
}

void GPIO::readEvent(unsigned int offset, LineEvent &event,
		unsigned long long &timestamp) const {
	gpio_v2_line_event e;
	auto nRead = ::read(getLine(offset), &e, sizeof(e));

	if (nRead == -1) {
		throw SystemException();
	}

	if (nRead != sizeof(e)) {
		throw Exception(EX_OPERATION);
	}

	if (e.id == GPIO_V2_LINE_EVENT_RISING_EDGE) {
		event = LineEvent::RISING;
	} else if (e.id == GPIO_V2_LINE_EVENT_FALLING_EDGE) {
		event = LineEvent::FALLING;
	} else {
		event = LineEvent::NONE;
	}
	timestamp = e.timestamp_ns;
}

void GPIO::write(unsigned int offset, bool active) const {
	gpio_v2_line_values data;
	::memset(&data, 0, sizeof(data));
	data.bits = active ? 1 : 0;
	data.mask = 1;

	if (ioctl(getLine(offset), GPIO_V2_LINE_SET_VALUES_IOCTL, &data) == -1) {
		throw SystemException();
	}
}

void GPIO::printInfo() const noexcept {
	try {
		gpiochip_info ci;
		::memset(&ci, 0, sizeof(ci));

		if (::ioctl(chip.get(), GPIO_GET_CHIPINFO_IOCTL, &ci) == -1) {
			throw SystemException();
		}
		printf("\n\n=========================================\n");
		printf("Chip name: %s\n", ci.name);
		printf("Chip label: %s\n", ci.label);
		printf("Number of lines: %u\n", ci.lines);

		for (unsigned int i = 0; i < ci.lines; ++i) {
			gpio_v2_line_info li;
			::memset(&li, 0, sizeof(li));
			li.offset = i;

			if (::ioctl(chip.get(), GPIO_V2_GET_LINEINFO_IOCTL, &li) == -1) {
				throw SystemException();
			}

			printf("\n-----------------------------------------\n");
			printf("Line offset: %u, name: %s, consumer: %s\n", li.offset,
					li.name, li.consumer);

			if (li.flags & GPIO_V2_LINE_FLAG_USED) {
				printf("Busy\n");
			}

			LineConfig lc;
			bool value = false;
			getConfiguration(li.flags, lc);
			for (unsigned int n = 0; n < li.num_attrs; ++n) {
				const auto &attr = li.attrs[n];
				if (attr.id == GPIO_V2_LINE_ATTR_ID_DEBOUNCE) {
					lc.debounce.on = true;
					lc.debounce.period = attr.debounce_period_us;
				} else if (attr.id == GPIO_V2_LINE_ATTR_ID_FLAGS) {
					getConfiguration(attr.flags, lc);
				} else {
					value = attr.values;
				}
			}

			printf("active: %s, attributes: %u\n", WH_BOOLF(value),
					li.num_attrs);
			printConfiguration(lc);
		}
	} catch (const BaseException &e) {
		fprintf(stderr, "Error: %s\n", e.what());
	} catch (...) {
		fprintf(stderr, "Unknown error\n");
	}
}

void GPIO::printConfiguration(const LineConfig &config) noexcept {
	uint64_t flags = getFlags(config);
	LineConfig lc = config;
	getConfiguration(flags, lc);
	::printConfiguration(stdout, lc);
}

bool GPIO::setLine(int fd, unsigned int offset) noexcept {
	if (fd <= 0) {
		return false;
	}

	auto oldfd = getLine(offset);
	if (oldfd == fd) {
		return true;
	}

	if (oldfd > 0) {
		::close(oldfd);
	}
	return lines.put(fd, offset);
}

void GPIO::resetLines(unsigned int count) {
	closeLines();

	if (count <= lines.capacity()) {
		lines.setLimit(count);
		return;
	}

	lines.initialize(count);
	for (unsigned int i = 0; i < lines.getLimit(); ++i) {
		lines.put(-1, i);
	}
}

unsigned int GPIO::readLinesCount() {
	gpiochip_info info;
	memset(&info, 0, sizeof(info));

	if (::ioctl(chip.get(), GPIO_GET_CHIPINFO_IOCTL, &info) == -1) {
		throw SystemException();
	}

	return info.lines;
}

} /* namespace wanhive */
