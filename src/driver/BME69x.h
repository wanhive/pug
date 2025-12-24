/*
 * BME69x.h
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

/*
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *
 * Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
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

#ifndef WH_DRIVER_BME69X_H_
#define WH_DRIVER_BME69X_H_
#include "../physical/SMBus.h"

namespace wanhive {
/**
 * Operating modes
 */
enum BME69XMode : unsigned char {
	BME69X_MODE_SLEEP = (0), /**< Sleep mode */
	BME69X_MODE_FORCED = (1), /**< Forced mode */
	BME69X_MODE_PARALLEL = (2), /**< Parallel mode */
	BME69X_MODE_SEQUENTIAL = (3)/**< Sequential mode */
};

/**
 * Oversampling settings
 */
enum BME69XOverSampling : unsigned char {
	BME69X_OS_NONE = (0),/**< Switch off the measurement */
	BME69X_OS_1X = (1), /**< Perform 1 measurement */
	BME69X_OS_2X = (2), /**< Perform 2 measurements */
	BME69X_OS_4X = (3), /**< Perform 4 measurements */
	BME69X_OS_8X = (4), /**< Perform 8 measurements */
	BME69X_OS_16X = (5) /**< Perform 16 measurements */
};
/**
 * IIR Filter settings
 */
enum BME69XFilter : unsigned char {
	BME69X_FILTER_OFF = (0), /**< Switch off the filter */
	BME69X_FILTER_SIZE_1 = (1), /**< Filter coefficient of 2 */
	BME69X_FILTER_SIZE_3 = (2), /**< Filter coefficient of 4 */
	BME69X_FILTER_SIZE_7 = (3), /**< BFilter coefficient of 8 */
	BME69X_FILTER_SIZE_15 = (4),/**< Filter coefficient of 16 */
	BME69X_FILTER_SIZE_31 = (5),/**< Filter coefficient of 32 */
	BME69X_FILTER_SIZE_63 = (6),/**< Filter coefficient of 64 */
	BME69X_FILTER_SIZE_127 = (7)/**< Filter coefficient of 128 */
};

/**
 * ODR/Standby time
 */
enum BME69XStandBy : unsigned char {
	BME69X_SB_0_59_MS = (0),/**< Standby time of 0.59ms */
	BME69X_SB_62_5_MS = (1),/**< Standby time of 62.5ms */
	BME69X_SB_125_MS = (2), /**< Standby time of 125ms */
	BME69X_SB_250_MS = (3), /**< Standby time of 250ms */
	BME69X_SB_500_MS = (4), /**< Standby time of 500ms */
	BME69X_SB_1000_MS = (5),/**< Standby time of 1s */
	BME69X_SB_10_MS = (6), /**< Standby time of 10ms */
	BME69X_SB_20_MS = (7), /**< Standby time of 20ms */
	BME69X_SB_NONE = (8) /**< No standby time */
};

/**
 * Over-sampling and filter settings
 */
struct BME69xConfig {
	/*! Over-sampling settings */
	struct {
		/*! Humidity over-sampling */
		BME69XOverSampling humidity;
		/*! Temperature over-sampling  */
		BME69XOverSampling temperature;
		/*! Pressure over-sampling */
		BME69XOverSampling pressure;
	} osr;

	/*! Filter coefficient */
	BME69XFilter filter;
	/*! Standby time between sequential mode measurement profiles */
	BME69XStandBy standby;
};

struct BME69xHeaterConfig {
	/*! Enable gas measurement */
	bool enable;
	/*! Heater temperature for forced mode degree Celsius */
	unsigned short temperature;
	/*! Heating duration for forced mode in milliseconds */
	unsigned short duration;

	/*! Profiles */
	struct {
		/*! Heater temperature profile in degree Celsius */
		unsigned short *temperature;
		/*! Heating duration profile in milliseconds */
		unsigned short *duration;
		/*! Length of the heating profile */
		unsigned char length;
		/*! Heating duration for parallel mode in milliseconds */
		unsigned short wait;
	} profile;
};

struct BME69xData {
	/*! Meta data */
	struct {
		/*! Contains new_data, gasm_valid & heat_stab */
		unsigned char status;
		/*! The index of the heater profile used */
		unsigned char step;
		/*! Measurement index to track order */
		unsigned char index;
		/*! Heater resistance */
		unsigned char resistance;
		/*! Current DAC */
		unsigned char idac;
		/*! Gas wait period */
		unsigned char period;
		/*! Intermediate temperature co-efficient */
		unsigned int tco;
	} meta;

	/*! Temperature in degree celsius x100 */
	short temperature;
	/*! Pressure in Pascal */
	unsigned int pressure;
	/*! Humidity in % relative humidity x1000 */
	unsigned int humidity;
	/*! Gas resistance in Ohms */
	unsigned int gas;
};

/**
 * User space driver for BME690 environment sensor.
 * @note Supports forced, parallel, and sequential modes over an I2C interface.
 * @ref https://github.com/boschsensortec/BME690_SensorAPI
 */
class BME69x: protected SMBus {
public:
	/**
	 * Constructor: initializes the sensor.
	 * @param bus i2c adapter's identifier
	 * @param address device identifier (typically 0x77)
	 */
	BME69x(unsigned int bus, unsigned int address = I2C_ADDR_HIGH);
	/**
	 * Constructor: initializes the sensor.
	 * @param path i2c adapter's pathname
	 * @param address device identifier (typically 0x77)
	 */
	BME69x(const char *path, unsigned int address = I2C_ADDR_HIGH);
	/**
	 * Destructor:  closes the i2c bus.
	 */
	~BME69x();
	/**
	 * Performs a soft-reset and initializes the sensor.
	 */
	void setup();
	/**
	 * Performs a soft reset.
	 */
	void reset() const;
	/**
	 * Reads sensor's current operation mode.
	 * @return operation mode
	 */
	BME69XMode getOperationMode() const;
	/**
	 * Sets sensor's operation mode.
	 * @param mode desired operation mode
	 */
	void setOperationMode(BME69XMode mode) const;
	/**
	 * Reads configuration data (over-sampling and filter) from the sensor.
	 * @param conf stores the configuration data
	 */
	void getConfiguration(BME69xConfig &conf) const;
	/**
	 * Writes new configuration data (over-sampling and filter) to the sensor.
	 * @param conf new configuration data
	 */
	void setConfiguration(const BME69xConfig &conf) const;
	/**
	 * Reads the sensor's gas-heater settings.
	 * @param conf stores the configuration data
	 */
	void getHeaterConfiguration(BME69xHeaterConfig &conf) const;
	/**
	 * Writes gas heater settings to the sensor.
	 * @param mode desired operation mode
	 * @param conf new configuration data
	 */
	void setHeaterConfiguration(BME69XMode mode,
			const BME69xHeaterConfig &conf) const;
	/**
	 * Returns the remaining duration that can be used for heating.
	 * @param mode desired operation mode
	 * @param conf sensor's configuration data
	 * @return duration in microseconds
	 */
	unsigned int getMeasurementDuration(BME69XMode mode,
			const BME69xConfig &conf) const noexcept;
	/**
	 * Sets the ambient temperature for defining the heater temperature.
	 * @param temperature ambient temperature
	 */
	void setAmbientTemperature(char temperature) noexcept;
	/**
	 * Returns sensor data in the forced mode.
	 * @param data stores the sensor data
	 * @return true if fresh data is available, false otherwise
	 */
	bool getData(BME69xData &data) const;
	/**
	 * Returns sensor data in sequential or parallel modes.
	 * @param data stores the sensor data
	 * @return number of available data instances.
	 */
	unsigned int getData(BME69xData (&data)[3]) const;
private:
	void readFieldData(unsigned char index, BME69xData &data) const;
	void readAllFieldData(BME69xData *(&data)[3]) const;
	void sortSensorData(unsigned lowIndex, unsigned highIndex,
			BME69xData *field[]) const noexcept;
	short calcTemperature(unsigned int eaw, unsigned int &tCoeff) const noexcept;
	unsigned int calcPressure(unsigned int raw,
			unsigned int tCoeff) const noexcept;
	unsigned int calcHumidity(unsigned short raw,
			short temperature) const noexcept;
	unsigned int calcGasResistance(unsigned short raw,
			unsigned char gasRange) const noexcept;
	void calibrate();
	void configureHeater(const BME69xHeaterConfig &conf, unsigned char mode,
			unsigned char &nConv) const;
	unsigned char calculateHeaterResistance(
			unsigned short temperature) const noexcept;
	unsigned char calculateGasWait(unsigned short duration) const noexcept;
	unsigned char calculateHeaterDurationShared(
			unsigned short duration) const noexcept;
	void writeRegisters(const unsigned char *commands,
			const unsigned char *values, unsigned int length) const;
public:
	/* BME69X unique chip identifier */
	static constexpr unsigned char CHIP_ID = (0x61);
	/* BME69X lower I2C address */
	static constexpr unsigned char I2C_ADDR_LOW = (0x76);
	/* BME69X higher I2C address */
	static constexpr unsigned char I2C_ADDR_HIGH = (0x77);
private:
	struct {
		unsigned char chip;
		unsigned char variant;
		char baseline;
	} dev;

	struct {
		struct {
			short par_h1;
			char par_h2;
			unsigned char par_h3;
			char par_h4;
			short par_h5;
			unsigned char par_h6;
		} hum;

		struct {
			char par_g1;
			short par_g2;
			char par_g3;
			unsigned char res_heat_range;
			char res_heat_val;
			char range_sw_err;
		} gas;

		struct {
			unsigned short par_t1;
			unsigned short par_t2;
			char par_t3;
		} temp;

		struct {
			short par_p5;
			short par_p6;
			char par_p7;
			char par_p8;
			unsigned short par_p1;
			unsigned short par_p2;
			char par_p3;
			char par_p4;
			short par_p9;
			char par_p10;
			char par_p11;
		} pres;

		int t_fine;
	} calib;
};

} /* namespace wanhive */

#endif /* WH_DRIVER_BME69X_H_ */
