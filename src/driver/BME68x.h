/*
 * BME68x.h
 *
 * Copyright (C) 2024 Wanhive Systems Private Limited (info@wanhive.com)
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
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
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
 *
 */

#ifndef WH_DRIVER_BME68X_H_
#define WH_DRIVER_BME68X_H_
#include "../physical/SMBus.h"

namespace wanhive {
/**
 * Over-sampling and filter settings.
 */
struct BME68xConfig {
	struct {
		/*! Humidity over-sampling */
		unsigned char humidity;
		/*! Temperature over-sampling */
		unsigned char temperature;
		/*! Pressure over-sampling */
		unsigned char pressure;
	} os;

	/*! Filter coefficient */
	unsigned char filter;
	/*! Standby time between sequential mode measurement profiles */
	unsigned char odr;
};

/**
 * Gas heater configuration.
 */
struct BME68xHeaterConfig {
	/*! Enable gas measurement */
	unsigned char enable;
	/*! Heater temperature for forced mode degree Celsius */
	unsigned short temperature;
	/*! Heating duration for forced mode in milliseconds */
	unsigned short duration;

	struct {
		/*! Heater temperature profile in degree Celsius */
		unsigned short *temperature;
		/*! Heating duration profile in milliseconds */
		unsigned short *duration;
		/*! Length of the heating profile */
		unsigned char length;
		/*! Heating duration for parallel mode in milliseconds */
		unsigned short sharedDuration;
	} profile;

};

/**
 * Sensor data.
 */
struct BME68xData {
	/*! Contains new_data, gasm_valid & heat_stab */
	unsigned char status;
	/*! The index of the heater profile used */
	unsigned char gasIndex;
	/*! Measurement index to track order */
	unsigned char measurementIndex;
	/*! Heater resistance */
	unsigned char heaterResistance;
	/*! Current DAC */
	unsigned char idac;
	/*! Gas wait period */
	unsigned char gasWait;

	/*! Temperature in degree celsius x100 */
	short temperature;
	/*! Pressure in Pascal */
	unsigned int pressure;
	/*! Humidity in % relative humidity x1000 */
	unsigned int humidity;
	/*! Gas resistance in Ohms */
	unsigned int gasResistance;
};

/**
 * User space driver for the BME68x (BME680, BME 688) environment sensors.
 * @note Supports forced, parallel, and sequential modes over an I2C interface.
 * @ref https://github.com/boschsensortec/BME68x_SensorAPI
 */
class BME68x: protected SMBus {
public:
	/**
	 * Constructor: initializes the sensor.
	 * @param bus i2c adapter's identifier
	 * @param address device identifier (typically 0x77)
	 */
	BME68x(unsigned int bus, unsigned int address = I2C_ADDR_HIGH);
	/**
	 * Constructor: initializes the sensor.
	 * @param path i2c adapter's pathname
	 * @param address device identifier (typically 0x77)
	 */
	BME68x(const char *path, unsigned int address = I2C_ADDR_HIGH);
	/**
	 * Destructor:  closes the i2c bus.
	 */
	~BME68x();
	/**
	 * Performs a soft-reset and initializes the sensor.
	 */
	void setup();
	/**
	 * Performs a soft reset.
	 */
	void reset() const;
	/**
	 * Reads configuration data (over-sampling and filter) from the sensor.
	 * @param config stores the configuration data
	 */
	void getConfiguration(BME68xConfig &config) const;
	/**
	 * Writes new configuration data (over-sampling and filter) to the sensor.
	 * @param config new configuration data
	 */
	void setConfiguration(const BME68xConfig &config);
	/**
	 * Reads the sensor's gas-heater settings.
	 * @param config stores the configuration data
	 */
	void getHeaterConfiguration(BME68xHeaterConfig &config) const;
	/**
	 * Writes gas heater settings to the sensor.
	 * @param opMode desired operation mode
	 * @param config new configuration data
	 */
	void setHeaterConfiguration(unsigned char opMode,
			const BME68xHeaterConfig &config);
	/**
	 * Returns the remaining duration that can be used for heating.
	 * @param opMode desired operation mode
	 * @param conf sensor's configuration data
	 * @return duration in microseconds
	 */
	unsigned int getMeasurementDuration(const unsigned char opMode,
			const BME68xConfig &conf) noexcept;
	/**
	 * Returns sensor data in the forced mode.
	 * @param data stores the sensor data
	 * @return true if fresh data is available, false otherwise
	 */
	bool getData(BME68xData &data);
	/**
	 * Returns sensor data in sequential or parallel modes.
	 * @param data stores the sensor data
	 * @return number of available data instances.
	 */
	unsigned int getData(BME68xData (&data)[3]);
	/**
	 * Reads sensor's current operation mode.
	 * @return operation mode
	 */
	unsigned char getOperationMode() const;
	/**
	 * Sets sensor's operation mode.
	 * @param mode desired operation mode
	 */
	void setOperationMode(unsigned char mode) const;
	/**
	 * Sets the ambient temperature for defining the heater temperature.
	 * @param temperature ambient temperature
	 */
	void setAmbientTemperature(char temperature) noexcept;
private:
	void calibrate();
	void configureHeater(const BME68xHeaterConfig &config, unsigned char opMode,
			unsigned char &nConv) const;
	void readFieldData(unsigned char index, BME68xData &data);
	void readAllFieldData(BME68xData *(&data)[3]);
	void sortSensorData(unsigned lowIndex, unsigned highIndex,
			BME68xData *field[]) const noexcept;
	short calculateTemperature(unsigned int raw) noexcept;
	unsigned int calculatePressure(unsigned int raw) const noexcept;
	unsigned int calculateHumidity(unsigned int raw) const noexcept;
	unsigned int calculateGasResistanceLow(unsigned short raw,
			unsigned char range) const noexcept;
	unsigned int calculateGasResistanceHigh(unsigned short raw,
			unsigned char range) const noexcept;
	unsigned char calculateHeaterResistance(
			unsigned short temperature) const noexcept;
	unsigned char calculateGasWait(unsigned short duration) const noexcept;
	unsigned char calculateHeaterDurationShared(
			unsigned short duration) const noexcept;
	void boundaryCheck(unsigned char &value, unsigned char max) noexcept;
	void writeRegisters(const unsigned char *commands,
			const unsigned char *values, unsigned int length) const;
public:
	/*! BME68X unique chip identifier */
	static constexpr unsigned char CHIP_ID = (0x61);
	/*! Low Gas variant */
	static constexpr unsigned char VARIANT_GAS_LOW = (0x00);
	/*! High Gas variant */
	static constexpr unsigned char VARIANT_GAS_HIGH = (0x01);
	/*! BME68X lower I2C address */
	static constexpr unsigned char I2C_ADDR_LOW = (0x76);
	/*! BME68X higher I2C address */
	static constexpr unsigned char I2C_ADDR_HIGH = (0x77);

	/**
	 * Enable/Disable
	 */
	enum Switch : unsigned char {
		SW_ENABLE = (0x01),/**< Enable */
		SW_DISABLE = (0x00)/**< Disable */
	};
	/**
	 * Oversampling setting
	 */
	enum Oversampling : unsigned char {
		OS_NONE = (0),/**< Switch off measurement */
		OS_1X = (1), /**< Perform 1 measurement */
		OS_2X = (2), /**< Perform 2 measurements */
		OS_4X = (3), /**< Perform 4 measurements */
		OS_8X = (4), /**< Perform 8 measurements */
		OS_16X = (5) /**< Perform 16 measurements */
	};
	/**
	 * IIR Filter settings
	 */
	enum Filter : unsigned char {
		FILTER_OFF = (0), /**< Switch off the filter */
		FILTER_SIZE_1 = (1), /**< Filter coefficient of 2 */
		FILTER_SIZE_3 = (2), /**< Filter coefficient of 4 */
		FILTER_SIZE_7 = (3), /**< Filter coefficient of 8 */
		FILTER_SIZE_15 = (4),/**< Filter coefficient of 16 */
		FILTER_SIZE_31 = (5),/**< Filter coefficient of 32 */
		FILTER_SIZE_63 = (6),/**< Filter coefficient of 64 */
		FILTER_SIZE_127 = (7)/**< Filter coefficient of 128 */
	};

	/**
	 * ODR/Standby time
	 */
	enum Standby : unsigned char {
		ODR_0_59_MS = (0),/**< Standby time of 0.59ms */
		ODR_62_5_MS = (1),/**< BStandby time of 62.5ms */
		ODR_125_MS = (2),/**< Standby time of 125ms */
		ODR_250_MS = (3),/**< Standby time of 250ms */
		ODR_500_MS = (4),/**< Standby time of 500ms */
		ODR_1000_MS = (5),/**< Standby time of 1s */
		ODR_10_MS = (6),/**< Standby time of 10ms */
		ODR_20_MS = (7),/**< Standby time of 20ms */
		ODR_NONE = (8) /**< No standby time */
	};

	/**
	 * Operating modes
	 */
	enum Mode : unsigned char {
		MODE_SLEEP = (0), /**< Sleep operation mode */
		MODE_FORCED = (1), /**< Forced operation mode */
		MODE_PARALLEL = (2), /**< Parallel operation mode */
		MODE_SEQUENTIAL = (3)/**< Sequential operation mode */
	};
private:
	struct {
		unsigned char chipId;
		unsigned char variantId;
		unsigned char info;
		char ambientTemperature;
	} dev;

	struct {
		struct {
			unsigned short par_t1;
			short par_t2;
			char par_t3;
		} temp;

		struct {
			unsigned short par_p1;
			short par_p2;
			char par_p3;
			short par_p4;
			short par_p5;
			char par_p6;
			char par_p7;
			short par_p8;
			short par_p9;
			unsigned char par_p10;
		} pres;

		struct {
			unsigned short par_h1;
			unsigned short par_h2;
			char par_h3;
			char par_h4;
			char par_h5;
			unsigned char par_h6;
			char par_h7;
		} hum;

		struct {
			char par_g1;
			short par_g2;
			char par_g3;
			unsigned char res_heat_range;
			char res_heat_val;
		} gas;

		char range_sw_err;
		int t_fine;

	} calib;
};

} /* namespace wanhive */

#endif /* WH_DRIVER_BME68X_H_ */
