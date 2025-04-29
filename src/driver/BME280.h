/*
 * BME280.h
 *
 * Copyright (C) 2023 Wanhive Systems Private Limited (info@wanhive.com)
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

/*
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef WH_DRIVER_BME280_H_
#define WH_DRIVER_BME280_H_
#include "../physical/SMBus.h"

namespace wanhive {
/**
 * Sensor power modes.
 */
enum BME280Mode : unsigned char {
	BME280_MODE_SLEEP = (0x00), /**< Sleep mode */
	BME280_MODE_FORCED = (0x01),/**< Forced mode */
	BME280_MODE_NORMAL = (0x03) /**< Normal mode */
};

/**
 * Oversampling rates.
 */
enum BME280OverSampling : unsigned char {
	BME280_OS_NONE = (0x00), /**< Skip */
	BME280_OS_1X = (0x01), /**< Perform 1 measurement */
	BME280_OS_2X = (0x02), /**< Perform 2 measurements */
	BME280_OS_4X = (0x03), /**< Perform 4 measurements */
	BME280_OS_8X = (0x04), /**< Perform 8 measurements */
	BME280_OS_16X = (0x05),/**< Perform 16 measurements */
	BME280_OS_MAX = (16) /**< For internal use */
};
/**
 * Filter coefficients.
 */
enum BME280Filter : unsigned char {
	BME280_FILTER_OFF = (0x00),/**< Switch off the filter */
	BME280_FILTER_2 = (0x01), /**< Filter coefficient of 2 */
	BME280_FILTER_4 = (0x02), /**< Filter coefficient of 4 */
	BME280_FILTER_8 = (0x03), /**< Filter coefficient of 8 */
	BME280_FILTER_16 = (0x04) /**< Filter coefficient of 16 */
};
/**
 * Standby durations (normal mode).
 */
enum BME280StandBy : unsigned char {
	BME280_SB_0_5_MS = (0x00), /**< Standby time of 0.5 ms */
	BME280_SB_62_5_MS = (0x01),/**< Standby time of 62.5 ms */
	BME280_SB_125_MS = (0x02), /**< Standby time of 125 ms */
	BME280_SB_250_MS = (0x03), /**< Standby time of 250 ms */
	BME280_SB_500_MS = (0x04), /**< Standby time of 500 ms */
	BME280_SB_1000_MS = (0x05),/**< Standby time of 1s */
	BME280_SB_10_MS = (0x06), /**< Standby time of 10ms */
	BME280_SB_20_MS = (0x07) /**< Standby time of 20ms */
};

/**
 * Over-sampling and filter settings.
 */
struct BME280Config {
	struct {
		/*! Pressure over-sampling */
		BME280OverSampling pressure;
		/*! Temperature over-sampling */
		BME280OverSampling temperature;
		/*! Humidity over-sampling */
		BME280OverSampling humidity;
	} osr;

	/*! Filter coefficient */
	BME280Filter filter;
	/*! Standby time */
	BME280StandBy standby;
};

/**
 * Raw sensor data.
 */
struct BME280RawData {
	/*! Pressure */
	unsigned pressure;
	/*! Temperature */
	unsigned temperature;
	/*! Humidity */
	unsigned humidity;
};

/**
 * Compensated sensor data.
 */
struct BME280Data {
	/*! Compensated pressure: Pascal */
	unsigned pressure;
	/*! Compensated temperature: degree celsius x100 */
	int temperature;
	/*! Compensated humidity: %RH x1000 */
	unsigned humidity;
};

/**
 * User space BME280 environment sensor driver.
 * @note Supports forced and normal power modes over an I2C interface.
 * @ref https://github.com/boschsensortec/BME280_SensorAPI
 */
class BME280: protected SMBus {
public:
	/**
	 * Constructor: initializes the sensor.
	 * @param bus i2c adapter's identifier
	 * @param address device identifier (typically 0x77)
	 */
	BME280(unsigned int bus, unsigned int address = I2C_ADDR_SEC);
	/**
	 * Constructor: initializes the sensor.
	 * @param path i2c adapter's pathname
	 * @param address device identifier (typically 0x77)
	 */
	BME280(const char *path, unsigned int address = I2C_ADDR_SEC);
	/**
	 * Destructor: closes the i2c bus.
	 */
	~BME280();
	/**
	 * Performs soft reset and initializes the sensor.
	 */
	void setup();
	/**
	 * Performs soft reset.
	 */
	void reset() const;
	/**
	 * Reads sensor's configuration data.
	 * @param conf stores the configuration data
	 */
	void getConfiguration(BME280Config &conf) const;
	/**
	 * Writes new configuration data to the sensor.
	 * @param conf configuration data
	 */
	void setConfiguration(const BME280Config &conf) const;
	/**
	 * Writes new configuration data to the sensor.
	 * @param conf configuration data
	 * @param what settings selector
	 */
	void setConfiguration(const BME280Config &conf, unsigned char what) const;
	/**
	 * Reads sensor's power mode.
	 * @return power mode
	 */
	BME280Mode getPowerMode() const;
	/**
	 * Writes new power mode to the sensor.
	 * @param mode power mode
	 */
	void setPowerMode(BME280Mode mode) const;
	/**
	 * Reads the status register's value.
	 * @return status code
	 */
	unsigned char getStatus() const;
	/**
	 * Reads sensor's data (pressure, temperature and humidity).
	 * @param data stores the compensated data
	 */
	void getData(BME280Data &data);
	/**
	 * Reads sensor's data (pressure, temperature and humidity).
	 * @param data stores the compensated data
	 * @param what component selector
	 */
	void getData(BME280Data &data, unsigned char what);
	/**
	 * Returns the maximum delay in microseconds required for the measurement
	 * to complete.
	 * @param conf over-sampling configuration data
	 * @return delay in microseconds
	 */
	unsigned calculateDelay(const BME280Config &conf) const noexcept;
private:
	void compensate(unsigned char what, const BME280RawData &raw,
			BME280Data &result) noexcept;
	void calibrate();
	void sleep() const;
	void writePowerMode(unsigned char mode) const;
	void parseConfiguration(const unsigned char *data,
			BME280Config &conf) const noexcept;
	void parseRawData(const unsigned char *data,
			BME280RawData &raw) const noexcept;
	void reload(const BME280Config &conf) const;
	void setOversampling(unsigned char desired, const BME280Config &conf) const;
	void setFilterAndStandby(unsigned char desired,
			const BME280Config &conf) const;
	void setHumidityOSR(const BME280Config &conf) const;
	void setTempPresOSR(unsigned char desired, const BME280Config &conf) const;
	int compensateTemperature(const BME280RawData &raw) noexcept;
	unsigned compensatePressure(const BME280RawData &raw) const noexcept;
	unsigned compensateHumidity(const BME280RawData &raw) const noexcept;
public:
	/*! BME280 chip identifier */
	static constexpr unsigned char CHIP_ID = (0x60);
	/*! BME280 lower I2C address */
	static constexpr unsigned char I2C_ADDR_PRIM = (0x76);
	/*! BME280 higher I2C address */
	static constexpr unsigned char I2C_ADDR_SEC = (0x77);
	/**
	 * Sensor component selection mask
	 */
	enum Component : unsigned char {
		SENSE_PRESSURE = (0x01), /**< Pressure selector */
		SENSE_TEMPERATURE = (0x02),/**< Temperature selector */
		SENSE_HUMIDITY = (0x04), /**< Humidity selector */
		SENSE_ALL = (0x07) /**< All components */
	};
	/**
	 * Settings selection mask
	 */
	enum Settings : unsigned char {
		SEL_OSR_PRESS = (1), /**< Pressure over-sampling */
		SEL_OSR_TEMP = (1 << 1), /**< Temperature over-sampling */
		SEL_OSR_HUM = (1 << 2), /**< Humidity over-sampling */
		SEL_FILTER = (1 << 3), /**< Filter */
		SEL_STANDBY = (1 << 4), /**< Standby time (normal mode) */
		SEL_ALL_SETTINGS = (0x1F)/**< All settings */
	};
	/**
	 * Status codes mask
	 */
	enum Status : unsigned char {
		STATUS_IM_UPDATE = (0x01),/**< NVM data being copied */
		STATUS_MEAS_DONE = (0x08) /**< Conversion running */
	};
private:
	struct {
		unsigned char chipId;
	} dev;

	struct {
		unsigned short dig_t1;
		short dig_t2;
		short dig_t3;

		unsigned short dig_p1;
		short dig_p2;
		short dig_p3;
		short dig_p4;
		short dig_p5;
		short dig_p6;
		short dig_p7;
		short dig_p8;
		short dig_p9;

		unsigned char dig_h1;
		short dig_h2;
		unsigned char dig_h3;
		short dig_h4;
		short dig_h5;
		char dig_h6;

		int t_fine;
	} calib;
};

} /* namespace wanhive */

#endif /* WH_DRIVER_BME280_H_ */
