/**
 * @file BMM350.h
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
 *
 */

#ifndef WH_DRIVER_BMM350_H_
#define WH_DRIVER_BMM350_H_
#include "../physical/SMBus.h"

/*! @namespace wanhive */
namespace wanhive {
/**
 * Enable or disable flags
 */
enum BMM350Switch : unsigned char {
	BMM350_DISABLE = (0x0),/**< Disable */
	BMM350_ENABLE = (0x1) /**< Enable */
};

/**
 * Power modes (use fast forced mode for data rate of 25Hz and higher).
 */
enum BMM350PowerMode : unsigned char {
	BMM350_MODE_SUSPEND = (0x00), /**< Suspend power mode */
	BMM350_MODE_NORMAL = (0x01), /**< Normal (auto) power mode */
	BMM350_MODE_FORCED = (0x03), /**< Forced (triggered) mode */
	BMM350_MODE_FAST = (0x04) /**< Forced fast data rate mode */
};

/**
 * Magnetic reset
 */
enum BMM350ResetType : unsigned char {
	BMM350_FLUXGUIDE_9MS = (0x05), /**< Flux-guide reset */
	BMM350_FLUXGUIDE_FAST = (0x06),/**< Fast flux-guide reset */
	BMM350_BITRESET_9MS = (0x07), /**< Bit reset */
	BMM350_BITRESET_FAST = (0x08), /**< Fast bit reset */
	BMM350_NOMAGRESET = (127) /**< No reset */
};

/**
 * Output data rates
 */
enum BMM350DataRate : unsigned char {
	BMM350_ODR_400HZ = (0x2), /**< 400Hz */
	BMM350_ODR_200HZ = (0x3), /**< 200Hz */
	BMM350_ODR_100HZ = (0x4), /**< 100Hz */
	BMM350_ODR_50HZ = (0x5), /**< 50Hz */
	BMM350_ODR_25HZ = (0x6), /**< 25Hz */
	BMM350_ODR_12_5HZ = (0x7), /**< 12.5Hz */
	BMM350_ODR_6_25HZ = (0x8), /**< 6.25Hz */
	BMM350_ODR_3_125HZ = (0x9),/**< 3.125Hz */
	BMM350_ODR_1_5625HZ = (0xA)/**< 1.5625HZ */
};

/**
 * Measurement averages
 */
enum BMM350SamplesAveraging : unsigned char {
	BMM350_AVERAGING_NONE = (0x0), /**< None */
	BMM350_AVERAGING_2 = (0x1), /**< 2 samples */
	BMM350_AVERAGING_4 = (0x2), /**< 4 samples */
	BMM350_AVERAGING_8 = (0x3), /**< 8 samples */
	BMM350_ULTRALOWNOISE = BMM350_AVERAGING_8,/**< Extremely low noise */
	BMM350_LOWNOISE = BMM350_AVERAGING_4, /**< Low noise */
	BMM350_REGULARPOWER = BMM350_AVERAGING_2, /**< Regular operation */
	BMM350_LOWPOWER = BMM350_AVERAGING_NONE /**< Low power operation */
};

/**
 * Interrupt mode
 */
enum BMM350InterruptLatch : unsigned char {
	BMM350_PULSED = (0x0),/**< Pulsed mode */
	BMM350_LATCHED = (0x1)/**< Latched mode */
};

/**
 * Interrupt polarity
 */
enum BMM350InterruptPolarity : unsigned char {
	BMM350_ACTIVE_LOW = (0x0),/**< Active low */
	BMM350_ACTIVE_HIGH = (0x1)/**< Active high */
};

/**
 * Interrupt output driver
 */
enum BMM350InterruptDrive : unsigned char {
	BMM350_INTR_OPEN_DRAIN = (0x0),/**< Open drain */
	BMM350_INTR_PUSH_PULL = (0x1) /**< Push-pull */
};

/**
 * Interrupt configuration
 */
struct BMM350InterruptConfig {
	/*! Interrupt mode */
	BMM350InterruptLatch latching { BMM350_PULSED };
	/*! Interrupt polarity */
	BMM350InterruptPolarity polarity { BMM350_ACTIVE_HIGH };
	/*! Interrupt output driver */
	BMM350InterruptDrive drivertype { BMM350_INTR_PUSH_PULL };
	/*! Enable INT pin */
	bool mapped { true };
};

/**
 * Performance configuration
 * @note Not all combinations are valid
 */
struct BMM350PerformanceConfig {
	/*! Output data rate */
	BMM350DataRate dataRate { BMM350_ODR_100HZ };
	/*! Samples averaging */
	BMM350SamplesAveraging averaging { BMM350_AVERAGING_4 };
};

/*!
 * Un-compensated (raw) data
 */
struct BMM350RawData {
	/*! Raw X data */
	int x;
	/*! Raw Y data */
	int y;
	/*! Raw Z data */
	int z;
	/*! Raw temperature value */
	int temperature;
};

/*!
 * Compensated data
 */
struct BMM350Data {
	/*! Compensated X data */
	float x;
	/*! Compensated Y data */
	float y;
	/*! Compensated Z data */
	float z;
	/*! Temperature data */
	float temperature;
};

/*!
 * PMU command status 0 structure
 */
struct BMM350PmuCmdStatus0 {
	/*! The previous PMU CMD is still in processing */
	unsigned char pmuCmdBusy;
	/*! The previous PMU_CMD_AGGR_SET.odr has been overwritten */
	unsigned char odrModified;
	/*! The previous PMU_CMD_AGGR_SET.avg has been overwritten */
	unsigned char avrModified;
	/*! The chip is in normal power mode */
	unsigned char normalPowerMode;
	/*! CMD value is not allowed */
	unsigned char illegalCmd;
	/*! Stores the latest PMU_CMD code processed */
	unsigned char pmuCmdValue;
};

/**
 * User space driver for the BMM350 magnetometer.
 * @note supports normal, forced and fast-forced modes over an I2C interface.
 * @ref https://github.com/boschsensortec/BMM350_SensorAPI
 */
class BMM350: protected SMBus {
public:
	/**
	 * Constructor: initializes the sensor
	 * @param bus i2c adapter's identifier
	 * @param address device identifier (typically 0x14)
	 */
	BMM350(unsigned int bus, unsigned int address = I2C_ADDR_LOW);
	/**
	 * Constructor: initializes the sensor
	 * @param path i2c adapter's pathname
	 * @param address device identifier (typically 0x14)
	 */
	BMM350(const char *path, unsigned int address = I2C_ADDR_LOW);
	/**
	 * Destructor
	 */
	~BMM350();

	/**
	 * Performs soft-reset and initializes the sensor.
	 */
	void setup();
	/**
	 * Performs soft reset (all registers are reset to default values).
	 */
	void reset();
	/**
	 * Obtains the interrupt status.
	 * @return data-ready interrupt status
	 */
	unsigned char getInterruptStatus();
	/**
	 * Sets sensor's power mode.
	 * @param mode desired power mode
	 */
	void setPowerMode(BMM350PowerMode mode);
	/**
	 * Updates data rate and noise performance settings. Please note that not
	 * all combinations are valid.
	 * @param ocfg desired performance settings
	 */
	void setPerformance(const BMM350PerformanceConfig &ocfg);
	/**
	 * Enables or disables magnetic measurement along axes.
	 * @param enableX true to enable the X-axis, false otherwise
	 * @param enableY true to enable the enable Y-axis, false otherwise
	 * @param enableZ true to enable the Z-axis, false otherwise
	 */
	void setAxes(bool enableX, bool enableY, bool enableZ);
	/**
	 * Reads sensor's time in ticks (when the last data was generated).
	 * @param seconds sensor time in seconds
	 * @param nanoseconds sensor time in nanoseconds
	 */
	void readSensorTime(unsigned int &seconds, unsigned int &nanoseconds);
	/**
	 * Enables or disables the data-ready interrupt.
	 * @param enable true to enable, false to disable
	 */
	void setInterrupt(bool enable);
	/**
	 * Configures the interrupt control settings.
	 * @param icfg interrupt configuration
	 */
	void configureInterrupt(const BMM350InterruptConfig &icfg);
	/**
	 * Reads the un-compensated sensor data.
	 * @param data stores the un-compensated data
	 */
	void readRawData(BMM350RawData &data);
	/**
	 * Configures the in-band-interrupt (IBI).
	 * @param enable true to enable the IBI, false to disable
	 * @param clearOnIBI true to clear the interrupt on IBI, false otherwise
	 */
	void setInterruptControlIBI(bool enable, bool clearOnIBI);
	/**
	 * Sets the pad drive strength.
	 * @param drive pad drive setting [0-7]
	 */
	void setPadDrive(unsigned char drive);
	/**
	 * Performs the magnetic reset of the sensor. This is necessary after a
	 * field shock (400mT field applied).
	 */
	void magneticResetAndWait();
	/**
	 * Reads the compensated sensor data.
	 * @param data stores the compensated data
	 */
	void readCompensatedData(BMM350Data &data);
	/**
	 * Configures the I2C watchdog timer.
	 * @param enable true to enable, false to disable
	 * @param longDelay true for long timeout, false for short timeout
	 */
	void setI2CWatchdogTimer(bool enable, bool longDelay);
	/**
	 * configures the sensor timer to run in suspend/forced mode.
	 * @param enable true to enable, false to disable
	 */
	void setCtrlUserRegister(bool enable);
	/**
	 * gets the PMU command status 0 value.
	 * @param status stores the status value
	 */
	void getPMUCommandStatus0(BMM350PmuCmdStatus0 &status);
protected:
	/**
	 * Reads a single byte from the sensor.
	 * @param command register selector
	 * @return register's value
	 */
	unsigned char readRegByte(unsigned char command);
	/**
	 * Reads a block of up to 32 bytes from the sensor.
	 * @param command register selector
	 * @param count number of bytes to read
	 * @param buffer stores the incoming data
	 * @return actual number of bytes read
	 */
	unsigned int readRegBytes(unsigned char command, unsigned int count,
			void *buffer);
private:
	void readOTPData();
	unsigned short readOTPWord(unsigned char addr);
	void updateOffsetAndSensitivity();
	void setPowerModeInternal(BMM350PowerMode mpde);
	void readOutRawData(float (&out_data)[4]);
public:
	/*! Low I2C address (default) */
	static constexpr unsigned char I2C_ADDR_LOW = (0x14);
	/*! High I2C address */
	static constexpr unsigned char I2C_ADDR_HIGH = (0x15);
	/*! Chip id of BMM350 */
	static constexpr unsigned char CHIP_ID = 0x33;
	/*! Minimum pad drive strength */
	static constexpr unsigned char PAD_DRIVE_WEAKEST = 0;
	/*! Maximum pad drive strength (default) */
	static constexpr unsigned char PAD_DRIVE_STRONGEST = 7;
private:
	static constexpr unsigned int OTP_DATA_LENGTH = 32;
	static constexpr unsigned int MAG_TEMP_DATA_LENGTH = 12;

	struct BMM350Compensate {
		struct {
			float t_offs;
			float offset_x;
			float offset_y;
			float offset_z;
		} dut_offset_coef;

		struct {
			float t_sens;
			float sens_x;
			float sens_y;
			float sens_z;
		} dut_sensit_coef;

		struct {
			float tco_x;
			float tco_y;
			float tco_z;
		} dut_tco;

		struct {
			float tcs_x;
			float tcs_y;
			float tcs_z;
		} dut_tcs;

		float dut_t0;

		struct {
			float cross_x_y;
			float cross_y_x;
			float cross_z_x;
			float cross_z_y;
		} cross_axis;
	};

	struct {
		unsigned char chip;
		unsigned char variant;
		unsigned char axes;
		unsigned short otp[OTP_DATA_LENGTH];
		BMM350Compensate compensate;
		bool autoBR;
	} dev;
};

} /* namespace wanhive */

#endif /* WH_DRIVER_BMM350_H_ */
