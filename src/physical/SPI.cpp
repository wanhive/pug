/*
 * SPI.cpp
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

#include "SPI.h"
#include <wanhive/base/common/Exception.h>
#include <wanhive/base/unix/FStat.h>
#include <wanhive/base/unix/SystemException.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

constexpr unsigned int SPI_MAJOR = 153;

const char* pathName(unsigned bus, unsigned device, char *buffer,
		unsigned size) {
	auto ret = snprintf(buffer, size, "/dev/spidev%u.%u", bus, device);
	if (ret > 0 && ((unsigned) ret < size)) {
		return buffer;
	} else {
		throw wanhive::Exception(wanhive::EX_OVERFLOW);
	}
}

uint32_t getMode(wanhive::SPIMode mode) noexcept {
	switch (mode) {
	case wanhive::SPIMode::MODE_0:
		return SPI_MODE_0;
	case wanhive::SPIMode::MODE_1:
		return SPI_MODE_1;
	case wanhive::SPIMode::MODE_2:
		return SPI_MODE_2;
	case wanhive::SPIMode::MODE_3:
		return SPI_MODE_3;
	default:
		return 0;
	}
}

wanhive::SPIMode getMode(uint32_t mode) noexcept {
	auto phase = (mode & SPI_CPHA) ? 0 : 1;
	auto polarity = (mode & SPI_CPOL) ? 0 : 2;
	switch (polarity + phase) {
	case 0:
		return wanhive::SPIMode::MODE_0;
	case 1:
		return wanhive::SPIMode::MODE_1;
	case 2:
		return wanhive::SPIMode::MODE_2;
	default:
		return wanhive::SPIMode::MODE_3;
	}
}

}  // namespace

namespace wanhive {

SPI::SPI(unsigned int bus, unsigned int device) {
	char buf[64];
	open(pathName(bus, device, buf, sizeof(buf)));
}

SPI::SPI(unsigned int bus, unsigned int device, const SPIConfig &cfg) {
	char buf[64];
	open(pathName(bus, device, buf, sizeof(buf)), cfg);
}

SPI::SPI(const char *path) {
	open(path);
}

SPI::SPI(const char *path, const SPIConfig &cfg) {
	open(path, cfg);
}

SPI::~SPI() {

}

void SPI::transfer(const void *tx, unsigned int txBytes, void *rx,
		unsigned int rxBytes) const {
	if (!tx && !txBytes && !rx && !rxBytes) {
		return;
	}

	struct spi_ioc_transfer sit[2];
	struct spi_ioc_transfer *sv = nullptr;
	unsigned int n = 0;

	if (tx && txBytes) {
		memset(&sit[0], 0, sizeof(sit[0]));
		sv = &sit[0];
		n += 1;
		sit[0].tx_buf = (__u64 ) tx;
		sit[0].len = txBytes;
	}

	if (rx && rxBytes) {
		memset(&sit[1], 0, sizeof(sit[1]));
		if (!sv) {
			sv = &sit[1];
		}
		n += 1;
		sit[1].rx_buf = (__u64 ) rx;
		sit[1].len = rxBytes;
	}

	if (n && (::ioctl(File::get(), SPI_IOC_MESSAGE(n), sv) == -1)) {
		throw SystemException();
	} else {
		return;
	}
}

void SPI::transfer(const void *tx, void *rx, unsigned int bytes) const {
	if ((!tx && !rx) || !bytes) {
		return;
	}

	struct spi_ioc_transfer sit;
	memset(&sit, 0, sizeof(sit));

	sit.tx_buf = (__u64 ) tx;
	sit.rx_buf = (__u64 ) rx;
	sit.len = bytes;

	if (::ioctl(File::get(), SPI_IOC_MESSAGE(1), &sit) == -1) {
		throw SystemException();
	} else {
		return;
	}
}

void SPI::getConfiguration(SPIConfig &cfg) const {
	uint32_t mode = 0;
	uint8_t bits = 0;
	uint32_t speed = 0;
	if (::ioctl(File::get(), SPI_IOC_RD_MODE32, &mode) == -1) {
		throw SystemException();
	} else if (ioctl(File::get(), SPI_IOC_RD_BITS_PER_WORD, &bits) == -1) {
		throw SystemException();
	} else if (ioctl(File::get(), SPI_IOC_RD_MAX_SPEED_HZ, &speed) == -1) {
		throw SystemException();
	} else {
		cfg.mode = getMode(mode);
		cfg.flags = mode & ~SPI_MODE_X_MASK;
		cfg.speed = speed;
		cfg.bits = bits;
	}
}

void SPI::setConfiguration(const SPIConfig &cfg) const {
	uint32_t speed = cfg.speed;
	uint8_t bits = cfg.bits;
	uint32_t mode = (cfg.flags & SPI_MODE_USER_MASK);
	mode &= ~SPI_MODE_X_MASK;
	mode |= getMode(cfg.mode);
	if (::ioctl(File::get(), SPI_IOC_WR_MODE32, &mode) == -1) {
		throw SystemException();
	} else if (::ioctl(File::get(), SPI_IOC_WR_BITS_PER_WORD, &bits) == -1) {
		throw SystemException();
	} else if (::ioctl(File::get(), SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1) {
		throw SystemException();
	} else {
		//success
	}
}

void SPI::open(const char *path) {
	try {
		File::open(path, O_RDWR);

		FStat stat(File::get());
		if (!(stat.isCharSpecialFile() && stat.majorDeviceId() == SPI_MAJOR)) {
			throw Exception(EX_OPERATION);
		}
	} catch (const BaseException &e) {
		File::close();
		throw;
	}
}

void SPI::open(const char *path, const SPIConfig &cfg) {
	try {
		open(path);
		setConfiguration(cfg);
	} catch (BaseException &e) {
		File::close();
		throw;
	}
}

} /* namespace wanhive */
