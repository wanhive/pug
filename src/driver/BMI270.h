/*
 * BMI270.h
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

#ifndef WH_DRIVER_BMI270_H_
#define WH_DRIVER_BMI270_H_
#include "../physical/SMBus.h"

namespace wanhive {

/**
 * Raw BMI270 sensor data.
 */
struct BMI270RawData {
	/*! X-axis */
	short x;
	/*! Y-axis */
	short y;
	/*! Z-axis */
	short z;
};

/**
 * Processed BMI270 sensor data.
 */
struct BMI270Data {
	/*! X-axis */
	double x;
	/*! Y-axis */
	double y;
	/*! Z-axis */
	double z;
};

/**
 * BMI270 power modes.
 */
enum BMI270PowerMode {
	BMI270_POWER_LOW, /**< Low power mode */
	BMI270_POWER_NORMAL, /**< Normal power mode */
	BMI270_POWER_PERFORMANCE/**< Performance mode */

};

/**
 * BMI270 accelerometer G-range.
 */
enum BMI270AccelerometerRange : unsigned char {
	BMI270_ACC_RANGE_2G = (0x00),/**< +-2G */
	BMI270_ACC_RANGE_4G = (0x01),/**< +-4G */
	BMI270_ACC_RANGE_8G = (0x02),/**< +-8G */
	BMI270_ACC_RANGE_16G = (0x03)/**< +-16G */
};

/**
 * BMI270 accelerometer output data rate.
 */
enum BMI270AccelerometerODR : unsigned char {
	BMI270_ACC_ODR_1600 = (0x0C),/**< 1600Hz */
	BMI270_ACC_ODR_800 = (0x0B), /**< 800Hz */
	BMI270_ACC_ODR_400 = (0x0A), /**< 400Hz */
	BMI270_ACC_ODR_200 = (0x09), /**< 200Hz */
	BMI270_ACC_ODR_100 = (0x08), /**< 100Hz */
	BMI270_ACC_ODR_50 = (0x07), /**< 50Hz */
	BMI270_ACC_ODR_25 = (0x06) /**< 25Hz */
};

/**
 * BMI270 accelerometer bandwidth parameter.
 */
enum BMI270AccelerometerBWP : unsigned char {
	BMI270_ACC_BWP_OSR4 = (0x00), /**< OSR4 mode, no averaging */
	BMI270_ACC_BWP_OSR2 = (0x01), /**< OSR2 mode, 2 samples */
	BMI270_ACC_BWP_NORMAL = (0x02),/**< Normal mode, 4 samples */
	BMI270_ACC_BWP_CIC = (0x03), /**< CIC mode, 8 samples */
	BMI270_ACC_BWP_RES16 = (0x04), /**< Reserved, 16 samples */
	BMI270_ACC_BWP_RES32 = (0x05), /**< Reserved, 32 samples */
	BMI270_ACC_BWP_RES64 = (0x06), /**< Reserved, 64 samples */
	BMI270_ACC_BWP_RES128 = (0x07) /**< Reserved, 128 samples */
};

/**
 * BMI270 gyroscope angular rate measurement range.
 */
enum BMI270GyroscopeRange : unsigned char {
	BMI270_GYR_RANGE_2000 = (0x00),/**< 2000 dps */
	BMI270_GYR_RANGE_1000 = (0x01),/**< 1000 dps */
	BMI270_GYR_RANGE_500 = (0x02), /**< 500 dps */
	BMI270_GYR_RANGE_250 = (0x03), /**< 250 dps */
	BMI270_GYR_RANGE_125 = (0x04) /**< 125 dps */
};

/**
 * BMI270 gyroscope output data rate.
 */
enum BMI270GyroscopeODR : unsigned char {
	BMI270_GYR_ODR_3200 = (0x0D),/**< 3200Hz */
	BMI270_GYR_ODR_1600 = (0x0C),/**< 1600Hz */
	BMI270_GYR_ODR_800 = (0x0B), /**< 800Hz */
	BMI270_GYR_ODR_400 = (0x0A), /**< 400Hz */
	BMI270_GYR_ODR_200 = (0x09), /**< 200Hz */
	BMI270_GYR_ODR_100 = (0x08), /**< 100Hz */
	BMI270_GYR_ODR_50 = (0x07), /**< 50Hz */
	BMI270_GYR_ODR_25 = (0x06) /**< 25Hz */
};

/**
 * BMI270 gyroscope bandwidth parameter.
 */
enum BMI270GyroscopeBWP : unsigned char {
	BMI270_GYR_BWP_OSR4 = (0x00), /**< OSR4 mode */
	BMI270_GYR_BWP_OSR2 = (0x01), /**< OSR2 mode */
	BMI270_GYR_BWP_NORMAL = (0x02)/**< Normal mode */
};

/**
 * User space driver for the BMI270 6-DOF IMU.
 * @note supports low-power, normal and performance modes over an I2C interface.
 */
class BMI270: protected SMBus {
public:
	/**
	 * Constructor: initializes the sensor
	 * @param bus i2c adapter's identifier
	 * @param address device identifier (typically 0x68)
	 */
	BMI270(unsigned int bus, unsigned int address = I2C_ADDR_LOW);
	/**
	 * Constructor: initializes the sensor
	 * @param path i2c adapter's pathname
	 * @param address device identifier (typically 0x68)
	 */
	BMI270(const char *path, unsigned int address = I2C_ADDR_LOW);
	/**
	 * Destructor
	 */
	~BMI270();
	/**
	 * Initializes the sensor.
	 */
	void setup();
	/**
	 * Sets sensor's power mode.
	 * @param mode desired power mode
	 */
	void setPowerMode(BMI270PowerMode mode);
	/**
	 * Enables/disables the auxiliary sensor.
	 * @param enable true to enable, false to disable
	 */
	void setAuxiliary(bool enable) const;
	/**
	 * Enables/disables the gyroscope.
	 * @param enable true to enable, false to disable
	 */
	void setGyroscope(bool enable) const;
	/**
	 * Enables/disables the accelerometer.
	 * @param enable true to enable, false to disable
	 */
	void setAccelerometer(bool enable) const;
	/**
	 * Enables/disables the temperature sensor.
	 * @param enable true to enable, false to disable
	 */
	void setTemperature(bool enable) const;
	/**
	 * Sets gyroscope's measurement range.
	 * @param range desired range
	 */
	void setGyroscopeRange(BMI270GyroscopeRange range);
	/**
	 * Sets accelerometer's G-range.
	 * @param range desired range
	 */
	void setAccelerometerRange(BMI270AccelerometerRange range);
	/**
	 * Sets gyroscope's output data rate.
	 * @param odr desired value
	 */
	void setGyroscopeODR(BMI270GyroscopeODR odr);
	/**
	 * Sets accelerometer's output data rate.
	 * @param odr desired value
	 */
	void setAccelerometerODR(BMI270AccelerometerODR odr);
	/**
	 * Sets gyroscope's bandwidth parameter.
	 * @param bwp desired value
	 */
	void setGyroscopeBWP(BMI270GyroscopeBWP bwp) const;
	/**
	 * Sets accelerometer's bandwidth parameter.
	 * @param bwp desired value
	 */
	void setAccelerometerBWP(BMI270AccelerometerBWP bwp) const;
	/**
	 * Enables/disables the FIFO header.
	 * @param enable true to enable, false to disable
	 */
	void setFIFOHeader(bool enable) const;
	/**
	 * Enables/disables streaming.
	 * @param enable true to enable, false to disable
	 */
	void setStreaming(bool enable) const;
	/**
	 * Sets gyroscope's noise performance.
	 * @param performance true for performance optimized, false for
	 * power optimized
	 */
	void setGyroscopeNoise(bool performance) const;
	/**
	 * Sets gyroscope's filter performance.
	 * @param performance true for performance optimized, false for
	 * power optimized
	 */
	void setGyroscopeFilter(bool performance) const;
	/**
	 * Sets accelerometer's filter performance.
	 * @param performance true for performance optimized, false for
	 * power optimized
	 */
	void setAccelerometerFilter(bool performance) const;
	/**
	 * Reads raw gyroscope data.
	 * @param data raw gyroscope data
	 */
	void getRawGyroscopeData(BMI270RawData &data) const;
	/**
	 * Reads raw accelerometer data.
	 * @param data accelerometer data
	 */
	void getRawAccelerometerData(BMI270RawData &data) const;
	/**
	 * Reads raw temperature data.
	 * @return raw temperature data
	 */
	short getRawTemperatureData() const;
	/**
	 * Reads gyroscope data in degree/s.
	 * @param data gyroscope data
	 */
	void getGyroscopeData(BMI270Data &data) const;
	/**
	 * Reads accelerometer data in m/s^2.
	 * @param data accelerometer data
	 */
	void getAccelerometerData(BMI270Data &data) const;
	/**
	 * Reads temperature data in degree centigrades.
	 * @return temperature data
	 */
	double getTemperatureData() const;
private:
	/**
	 * Loads sensor's configuration.
	 */
	void loadConfiguration();
public:
	/*! Low I2C address (default) */
	static constexpr unsigned char I2C_ADDR_LOW = (0x68);
	/*! High I2C address */
	static constexpr unsigned char I2C_ADDR_HIGH = (0x69);
private:
	/*! Default Chip Id */
	static constexpr unsigned char CHIP_ID = 0x24;
	struct {
		/* Chip ID */
		unsigned char chipId;
		/* Internal Status */
		unsigned char status;
		/* Accelerator Range */
		double accRange;
		/* Accelerator ODR */
		int accOdr;
		/* Gyroscope Range */
		double gyroRange;
		/* Gyroscope ODR */
		int gyroOdr;
	} dev;
};

} /* namespace wanhive */

#endif /* WH_DRIVER_BMI270_H_ */
