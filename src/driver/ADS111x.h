/*
 * ADS111x.h
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

#ifndef WH_DRIVER_ADS111X_H_
#define WH_DRIVER_ADS111X_H_
#include "../physical/SMBus.h"

namespace wanhive {
/**
 * ADS111x configuration data [15:0].
 */
struct ADS111xConfig {
	unsigned short status :1; //OS[15]
	unsigned short mux :3; //MUX[14:12]
	unsigned short gain :3; //PGA[11:9]
	unsigned short mode :1; //MODE[8]
	unsigned short rate :3; //DR[7:5]
	unsigned short cmode :1; //COMP_MODE[4]
	unsigned short polarity :1; //COMP_POL[3]
	unsigned short latch :1; //COMP_LAT[2]
	unsigned short queue :2; //COMP_QUE[1:0]
};

/**
 * User space ADS111x (ADS1113, ADS1114, ADS1115) driver.
 * @note ADS111x are I2C compatible 16-bit analog-to-digital converters (ADCs).
 * @ref https://www.ti.com/lit/gpn/ads1115
 */
class ADS111x: protected SMBus {
public:
	/**
	 * Constructor: initializes the driver.
	 * @param bus i2c adapter's identifier
	 * @param address device identifier (typically 0x48)
	 */
	ADS111x(unsigned int bus, unsigned int address = 0x48);
	/**
	 * Constructor: initializes the driver.
	 * @param path i2c adapter's pathname
	 * @param address device identifier (typically 0x48)
	 */
	ADS111x(const char *path, unsigned int address = 0x48);
	/**
	 * Destructor: closes the i2c bus.
	 */
	~ADS111x();
	//-----------------------------------------------------------------
	/**
	 * Reads from the conversion register.
	 * @return 16-bit value
	 */
	unsigned short getConversion() const;
	/**
	 * Reads from the config register.
	 * @return 16-bit value
	 */
	unsigned short getConfiguration() const;
	/**
	 * Writes into the config register.
	 * @param value new 16-bit value
	 */
	void setConfiguration(unsigned short value) const;
	/**
	 * Reads from the lo_thresh register.
	 * @return 16-bit value
	 */
	unsigned short getLowThreshold() const;
	/**
	 * Writes into the lo_thresh register.
	 * @param value new 16-bit value
	 */
	void setLowThreshold(unsigned short value) const;
	/**
	 * Reads from the hi_thresh register.
	 * @return 16-bit value
	 */
	unsigned short getHighThreshold() const;
	/**
	 * Writes into the hi_thresh register.
	 * @param value new 16-bit value
	 */
	void setHighThreshold(unsigned short value) const;
	//-----------------------------------------------------------------
	/**
	 * Decodes the configuration data.
	 * @param value 16-bit encoded value
	 * @param config stores the decoded configuration data
	 */
	static void decode(unsigned short value, ADS111xConfig &config) noexcept;
	/**
	 * Encodes the configuration data.
	 * @param config configuration data
	 * @return 16-bit encoded value
	 */
	static unsigned short encode(const ADS111xConfig &config) noexcept;
public:
	/**
	 * Default configuration.
	 */
	static constexpr unsigned short DEFAULT_CFG = 0x8583;
	/**
	 * Commonly used continuous-conversion mode configuration.
	 */
	static constexpr unsigned short WINDOW_CFG = 0x429A;
};

} /* namespace wanhive */

#endif /* WH_DRIVER_ADS111X_H_ */
