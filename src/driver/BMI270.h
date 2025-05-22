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
 * BMI270 sensors.
 */
enum BMI270Sensor : unsigned char {
	BMI270_AUX = (0x01), /**< Auxiliary */
	BMI270_GYR = (0x02), /**< Gyroscope */
	BMI270_ACC = (0x04), /**< Accelerometer */
	BMI270_TEMP = (0x08) /**< Temperature */
};

/**
 * BMI270 interrupt pins.
 */
enum BMI270IntPin : unsigned char {
	BMI270_INT1 = (0x53), /**< INT1 pin */
	BMI270_INT2 = (0x54) /**< INT2 pin */
};

/**
 * BMI270 power modes.
 */
enum BMI270PowerMode : unsigned char {
	BMI270_MODE_LP, /**< Low power mode */
	BMI270_MODE_NORMAL, /**< Normal power mode */
	BMI270_MODE_PERF /**< Performance mode */
};

/**
 * BMI270 accelerometer output data rate.
 */
enum BMI270AccelerometerODR : unsigned char {
	BMI270_ACC_ODR_0P78 = (0x01), /**< (25/32)Hz */
	BMI270_ACC_ODR_1P5 = (0x02), /**< (25/16)Hz */
	BMI270_ACC_ODR_3P1 = (0x03), /**< (25/8)Hz */
	BMI270_ACC_ODR_6P25 = (0x04), /**< (25/2)Hz */
	BMI270_ACC_ODR_12P5 = (0x05), /**< 25Hz */
	BMI270_ACC_ODR_25 = (0x06), /**< 25Hz */
	BMI270_ACC_ODR_50 = (0x07), /**< 50Hz */
	BMI270_ACC_ODR_100 = (0x08), /**< 100Hz */
	BMI270_ACC_ODR_200 = (0x09), /**< 200Hz */
	BMI270_ACC_ODR_400 = (0x0A), /**< 400Hz */
	BMI270_ACC_ODR_800 = (0x0B), /**< 800Hz */
	BMI270_ACC_ODR_1600 = (0x0C) /**< 1600Hz */
};

/**
 * BMI270 accelerometer bandwidth parameter.
 */
enum BMI270AccelerometerBWP : unsigned char {
	BMI270_ACC_BWP_OSR4 = (0x00), /**< OSR4 mode, no averaging */
	BMI270_ACC_BWP_OSR2 = (0x01), /**< OSR2 mode, 2 samples */
	BMI270_ACC_BWP_NORMAL = (0x02), /**< Normal mode, 4 samples */
	BMI270_ACC_BWP_CIC = (0x03), /**< CIC mode, 8 samples */
	BMI270_ACC_BWP_RES16 = (0x04), /**< Reserved, 16 samples */
	BMI270_ACC_BWP_RES32 = (0x05), /**< Reserved, 32 samples */
	BMI270_ACC_BWP_RES64 = (0x06), /**< Reserved, 64 samples */
	BMI270_ACC_BWP_RES128 = (0x07) /**< Reserved, 128 samples */
};

/**
 * BMI270 accelerometer G-range.
 */
enum BMI270AccelerometerRange : unsigned char {
	BMI270_ACC_RANGE_2G = (0x00), /**< +-2G */
	BMI270_ACC_RANGE_4G = (0x01), /**< +-4G */
	BMI270_ACC_RANGE_8G = (0x02), /**< +-8G */
	BMI270_ACC_RANGE_16G = (0x03) /**< +-16G */
};

/**
 * BMI270 gyroscope output data rate.
 */
enum BMI270GyroscopeODR : unsigned char {
	BMI270_GYR_ODR_25 = (0x06), /**< 25Hz */
	BMI270_GYR_ODR_50 = (0x07), /**< 50Hz */
	BMI270_GYR_ODR_100 = (0x08), /**< 100Hz */
	BMI270_GYR_ODR_200 = (0x09), /**< 200Hz */
	BMI270_GYR_ODR_400 = (0x0A), /**< 400Hz */
	BMI270_GYR_ODR_800 = (0x0B), /**< 800Hz */
	BMI270_GYR_ODR_1600 = (0x0C), /**< 1600Hz */
	BMI270_GYR_ODR_3200 = (0x0D) /**< 3200Hz */
};

/**
 * BMI270 gyroscope bandwidth parameter.
 */
enum BMI270GyroscopeBWP : unsigned char {
	BMI270_GYR_BWP_OSR4 = (0x00), /**< OSR4 mode */
	BMI270_GYR_BWP_OSR2 = (0x01), /**< OSR2 mode */
	BMI270_GYR_BWP_NORMAL = (0x02) /**< Normal mode */
};

/**
 * BMI270 gyroscope angular rate measurement range.
 */
enum BMI270GyroscopeRange : unsigned char {
	BMI270_GYR_RANGE_2000 = (0x00), /**< 2000 dps */
	BMI270_GYR_RANGE_1000 = (0x01), /**< 1000 dps */
	BMI270_GYR_RANGE_500 = (0x02), /**< 500 dps */
	BMI270_GYR_RANGE_250 = (0x03), /**< 250 dps */
	BMI270_GYR_RANGE_125 = (0x04) /**< 125 dps */

};

/**
 * BMI270 auxiliary sensor data rate.
 */
enum BMI270AuxOdr : unsigned char {
	BMI270_AUX_ODR_0P78 = (0x01), /**< (25/32)Hz */
	BMI270_AUX_ODR_1P5 = (0x02), /**< (25/16)Hz */
	BMI270_AUX_ODR_3P1 = (0x03), /**< (25/8)Hz */
	BMI270_AUX_ODR_6P25 = (0x04), /**< (25/2)Hz */
	BMI270_AUX_ODR_12P5 = (0x05), /**< 25Hz */
	BMI270_AUX_ODR_25 = (0x06), /**< 25Hz */
	BMI270_AUX_ODR_50 = (0x07), /**< 50Hz */
	BMI270_AUX_ODR_100 = (0x08), /**< 100Hz */
	BMI270_AUX_ODR_200 = (0x09), /**< 200Hz */
	BMI270_AUX_ODR_400 = (0x0A), /**< 400Hz */
	BMI270_AUX_ODR_800 = (0x0B), /**< 800Hz */
};

/**
 * BMI270 accelerometer configuration.
 */
struct BMI270AccelerometerConfig {
	/*! Output data rate */
	BMI270AccelerometerODR odr;
	/*! Bandwidth parameter */
	BMI270AccelerometerBWP bwp;
	/*! Filter performance */
	bool filter;
};

/**
 * BMI270 gyroscope configuration
 */
struct BMI270GyroscopeConfig {
	/*! Output data rate */
	BMI270GyroscopeODR odr;
	/*! Bandwidth parameter */
	BMI270GyroscopeBWP bwp;
	/*! Noise performance */
	bool noise;
	/*! Filter performance */
	bool filter;
};

/**
 * BMI270 auxiliary configuration.
 */
struct BMI270AuxiliaryConfig {
	/*! Output data rate */
	BMI270AuxOdr odr;
	/*! Trigger-readout offset (x2.5ms) */
	unsigned char offset;
};

/**
 * BMI270 interrupt pin electrical behavior.
 */
struct BMI270IntPinConfig {
	/*! Pin identifier */
	BMI270IntPin pin;
	/*! Output level */
	bool activeHigh;
	/*! Output behavior */
	bool openDrain;
	/*! Output enable */
	bool out;
	/*! Input enable */
	bool in;
};

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
 * BMI270 accelerometer and gyroscope ranges.
 */
struct BMI270Range {
	/*! Accelerometer's G-range */
	double g;
	/*! Gyroscope's DPS range */
	double dps;
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
 * User space driver for the BMI270 6-DOF IMU.
 * @note supports low-power, normal and performance modes over an I2C interface.
 */
class BMI270: protected SMBus {
public:
	/**
	 * Constructor: initializes the sensor.
	 * @param bus i2c adapter's identifier
	 * @param address device identifier (typically 0x68)
	 */
	BMI270(unsigned int bus, unsigned int address = I2C_ADDR_LOW);
	/**
	 * Constructor: initializes the sensor.
	 * @param path i2c adapter's pathname
	 * @param address device identifier (typically 0x68)
	 */
	BMI270(const char *path, unsigned int address = I2C_ADDR_LOW);
	/**
	 * Destructor: closes the i2c bus.
	 */
	~BMI270();
	/**
	 * Initializes the sensor.
	 */
	void setup();
	/**
	 * Performs soft-reset and writes the configuration file.
	 */
	void reset();
	/**
	 * Enables/disables advance power save mode.
	 * @param enable true to enable, false to disable
	 */
	void setAdvancePowerSave(bool enable) const;
	/**
	 * Gets the status of advance power saver mode
	 * @return true if enabled, false if disabled
	 */
	bool isAdvancePowerSave() const;
	/**
	 * Enables/disables fast power up mode.
	 * @param enable true to enable, false to disable
	 */
	void setFastPowerUp(bool enable) const;
	/**
	 * Gets the status of fast power up mode.
	 * @return true if enabled, false if disabled.
	 */
	bool isFastPowerUp() const;
	/**
	 * Sets sensor's power mode.
	 * @param mode desired power mode
	 */
	void setPowerMode(BMI270PowerMode mode) const;
	/**
	 * Enables/disables a sensor/feature.
	 * @param sensor sensor selector
	 * @param enable true to enable, false to disable
	 */
	void setSensor(BMI270Sensor sensor, bool enable) const;
	/**
	 * Gets a sensor's enabled/disabled status.
	 * @param sensor sensor selector
	 * @return true if enabled, false if disabled
	 */
	bool isSensor(BMI270Sensor sensor) const;
	/**
	 * Updates accelerometer's configuration.
	 * @param config new configuration
	 */
	void setAccelerometerConfiguration(
			const BMI270AccelerometerConfig &config) const;
	/**
	 * Reads accelerometer's configuration.
	 * @param config current configuration
	 */
	void getAccelerometerConfiguration(BMI270AccelerometerConfig &config) const;
	/**
	 * Sets accelerometer's G-range.
	 * @param range desired range
	 */
	void setAccelerometerRange(BMI270AccelerometerRange range) const;
	/**
	 * Reads accelerometer's G-range.
	 * @return current range
	 */
	BMI270AccelerometerRange getAccelerometerRange() const;
	/**
	 * Updates gyroscope's configuration.
	 * @param config new configuration
	 */
	void setGyroscopeConfiguration(const BMI270GyroscopeConfig &config) const;
	/**
	 * Reads gyroscope's configuration.
	 * @param config current configuration
	 */
	void getGyroscopeConfiguration(BMI270GyroscopeConfig &config) const;
	/**
	 * Sets gyroscope's measurement range.
	 * @param range desired range
	 */
	void setGyroscopeRange(BMI270GyroscopeRange range) const;
	/**
	 * Reads gyroscope's range.
	 * @return current range
	 */
	BMI270GyroscopeRange getGyroscopeRange() const;
	/**
	 * Updates auxiliary sensor interface's configuration.
	 * @param config new configuration
	 */
	void setAuxConfiguration(const BMI270AuxiliaryConfig &config) const;
	/**
	 * Reads auxiliary sensor interface's configuration.
	 * @param config current configuration
	 */
	void getAuxConfiguration(BMI270AuxiliaryConfig &config) const;
	/**
	 * Maps features to an interrupt pin.
	 * @param pin interrupt pin selector
	 * @param value features bitmap
	 */
	void setFeaturesMap(BMI270IntPin pin, unsigned char value) const;
	/**
	 * Reads features map of an interrupt pin.
	 * @param pin interrupt pin selector
	 * @return features bitmap
	 */
	unsigned char getFeaturesMap(BMI270IntPin pin) const;
	/**
	 * Maps sensor data to the interrupt pins.
	 * @param value sensor data bitmap
	 */
	void setDataMap(unsigned char value) const;
	/**
	 * Reads sensor data map of the interrupt pins.
	 * @return sensor data bitmap
	 */
	unsigned char getDataMap() const;
	/**
	 * Updates interrupt pin's configuration.
	 * @param config new configuration
	 */
	void setIntPinConfiguration(const BMI270IntPinConfig &config) const;
	/**
	 * Reads interrupt pin's configuration.
	 * @param config value-result argument, holds the pin identifier as
	 * the input parameter.
	 */
	void getIntPinConfiguration(BMI270IntPinConfig &config) const;
	/**
	 * Sets interrupt latch mode.
	 * @param enable true for permanent latched, false for non latched
	 */
	void setLatched(bool enable) const;
	/**
	 * Reads interrupt latch mode.
	 * @return true if permanent latched, false if non latched
	 */
	bool isLatched() const;
	/**
	 * Reads the sensor status.
	 * @return status code
	 */
	unsigned char getSensorStatus() const;
	/**
	 * Reads the internal status register.
	 * @return internal status and error bits
	 */
	unsigned char getInternalStatus() const;
	/**
	 * Reads the interrupt status.
	 * @return interrupt status code
	 */
	unsigned short getInterruptStatus() const;
	/**
	 * Reads the saturation status.
	 * @return status code
	 */
	unsigned char getSaturationStatus() const;
	/**
	 * Reads the drive strength.
	 * @return current drive strength
	 */
	unsigned char getDriveStrength() const;
	/**
	 * Sets the drive strength.
	 * @param value desired drive strength
	 */
	void setDriveStrength(unsigned char value);
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
	 * Reads raw accelerometer and gyroscope data.
	 * @param acc raw accelerometer data
	 * @param gyro raw gyroscope data
	 */
	void getRawData(BMI270RawData &acc, BMI270RawData &gyro) const;
	/**
	 * Reads raw temperature data.
	 * @return raw temperature data
	 */
	short getRawTemperatureData() const;
private:
	void writeConfiguration();
	void setFeature(unsigned char command, unsigned char feature,
			bool enable) const;
	bool isFeature(unsigned char command, unsigned char feature) const;
public:
	/*! Low I2C address (default) */
	static constexpr unsigned char I2C_ADDR_LOW = (0x68);
	/*! High I2C address */
	static constexpr unsigned char I2C_ADDR_HIGH = (0x69);
	/*! BMI270 chip identifier */
	static constexpr unsigned char CHIP_ID = 0x24;
private:
	struct {
		/* Chip ID */
		unsigned char chipId;
	} dev;
};

} /* namespace wanhive */

#endif /* WH_DRIVER_BMI270_H_ */
