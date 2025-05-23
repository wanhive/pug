/*
 * MLX90640.h
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

#ifndef WH_DRIVER_MLX90640_H_
#define WH_DRIVER_MLX90640_H_
#include "MLX9064x.h"

namespace wanhive {

/**
 * MLX90640 sub pages.
 */
enum MLX90640SubPage : unsigned char {
	MLX90640_SP0 = (0x00),/**< Sub page 0 */
	MLX90640_SP1 = (0x01) /**< Sub page 1 */
};

/**
 * MLX90640 refresh rates.
 */
enum MLX90640RefreshRate : unsigned char {
	MLX90640_RR_0_5 = (0x00),/**< 0.5Hz */
	MLX90640_RR_1 = (0x01), /**< 1Hz */
	MLX90640_RR_2 = (0x02), /**< 2Hz */
	MLX90640_RR_4 = (0x03), /**< 4Hz */
	MLX90640_RR_8 = (0x04), /**< 8Hz */
	MLX90640_RR_16 = (0x05), /**< 16Hz */
	MLX90640_RR_32 = (0x06), /**< 32Hz */
	MLX90640_RR_64 = (0x07) /**< 64Hz */
};

/**
 * MLX90640 resolutions (in bits).
 */
enum MLX90640Resolution : unsigned char {
	MLX90640_RES_16 = (0x00),/**< 16 bits */
	MLX90640_RES_17 = (0x01),/**< 17 bits */
	MLX90640_RES_18 = (0x02),/**< 18 bits */
	MLX90640_RES_19 = (0x03) /**< 19 bits */
};

/**
 * MLX90640 reading patterns.
 */
enum MLX90640Pattern : unsigned char {
	MLX90640_TV = (0x00),/**< Interleaved (TV) mode */
	MLX90640_CHESS = (0x01) /**< Chess pattern mode */
};

/**
 * MLX90640 pixel defects status
 */
enum MLX90640Defect : unsigned char {
	MLX90640_PIX_GOOD = (0x00), /**< No defects */
	MLX90640_PIX_OK = (0x01), /**< Within threshold */
	MLX90640_PIX_BROKEN = (0x02),/**< No output */
	MLX90640_PIX_OUTLIER = (0x04), /**< Out of specification */
	MLX90640_PIX_BAD = (0x06), /**< Defective pixels */
	MLX90640_PIX_ADJACENT = (0x08) /**< Adjacent defects */
};

struct MLX90640Config {
	/*! Activate/deactivate subpage mode */
	bool subpage;
	/*! Enable/Disable data hold */
	bool hold;
	/*! Repeat/Toggle the sub pages */
	bool repeat;
	/*! Refresh rate */
	MLX90640RefreshRate refreshRate;
	/*! ADC resolution */
	MLX90640Resolution resolution;
	/*! Reading pattern */
	MLX90640Pattern pattern;
};

/**
 * MLX90640 frame data structure.
 */
struct MLX90640Frame {
	/*! Raw frame data */
	uint16_t data[834];
	/*! Supply voltage */
	float vdd;
	/*! Ambient temperature */
	float ta;
};

/**
 * MLX90640 thermal data structure.
 */
struct MLX90640Data {
	/*! Processed data */
	float data[768];
	/*! Sub page number */
	MLX90640SubPage page;
};

/**
 * MLX90640 IR thermal camera driver.
 * @note Uses the I2C general-call reset command for soft-reset.
 */
class MLX90640: protected MLX9064x {
public:
	/**
	 * Constructor: initializes the sensor.
	 * @param bus i2c adapter's identifier
	 * @param address device identifier (typically 0x33)
	 */
	MLX90640(unsigned int bus, unsigned int address = I2C_ADDR);
	/**
	 * Constructor: initializes the sensor.
	 * @param path i2c adapter's pathname
	 * @param address device identifier (typically 0x33)
	 */
	MLX90640(const char *path, unsigned int address = I2C_ADDR);
	/**
	 * Destructor: closes the i2c bus.
	 */
	~MLX90640();
	/**
	 * Initializes the sensor and performs soft-reset.
	 */
	void setup();
	/**
	 * Performs soft reset and triggers fresh measurement cycle.
	 */
	void reset();
	/**
	 * Updates sensor's configuration (control register 1).
	 * @param config desired configuration.
	 */
	void setConfiguration(const MLX90640Config &config) const;
	/**
	 * Reads sensor's configuration (control register 1).
	 * @param config current configuration
	 */
	void getConfiguration(MLX90640Config &config) const;
	/**
	 * Selects a sub page for measurement (if sub pages toggle is disabled).
	 * @param page sub page's identifier
	 */
	void selectSubPage(MLX90640SubPage page) const;
	/**
	 * Starts a new measurement (clears status register's data available bit).
	 */
	void begin() const;
	/**
	 * Finishes the current measurement (indicate that the data has been read).
	 */
	void finish() const;
	/**
	 * Busy waits for new data to become available.
	 */
	void synchronize() const;
	/**
	 * Reads frame data (incl. auxiliary data and parameters) from the device.
	 * @param frame stores the frame data
	 * @return true on successful reading, false if data not available
	 */
	bool readFrame(MLX90640Frame &frame) const;
	/**
	 * Calculates object temperatures for all the pixels in a frame.
	 * @param frame frame data
	 * @param emissivity user-defined emissivity
	 * @param tReflected user-defined reflected temperature
	 * @param result stores the object temperatures
	 */
	void getTemperature(const MLX90640Frame &frame, float emissivity,
			float tReflected, MLX90640Data &result) const noexcept;
	/**
	 * Generates a thermal image for all the pixels in a frame.
	 * @param frame frame data
	 * @param result stores the output image
	 */
	void getImage(const MLX90640Frame &frame,
			MLX90640Data &result) const noexcept;
	/**
	 * Corrects the values of the broken pixels.
	 * @param pattern applicable working mode
	 * @param data thermal data array (value-result argument)
	 */
	void fixBroken(MLX90640Pattern pattern, MLX90640Data &data) const noexcept;
	/**
	 * Corrects the values of the outlier pixels.
	 * @param pattern applicable working mode
	 * @param data thermal data array (value-result argument)
	 */
	void fixOutlier(MLX90640Pattern pattern, MLX90640Data &data) const noexcept;
	/**
	 * Returns the pixel defect type.
	 * @return defect type
	 */
	MLX90640Defect getDefect() const noexcept;
	/**
	 * Returns the maximum delay in milliseconds required for a measurement
	 * to complete on the basis of the given refresh rate.
	 * @param refreshRate device's refresh rate
	 * @return delay in milliseconds
	 */
	static unsigned int calculateDelay(MLX90640RefreshRate refreshRate) noexcept;
	/**
	 * Extracts the given frame's subpage number.
	 * @param frame frame data
	 * @return subpage number (0 or 1)
	 */
	static MLX90640SubPage getSubPage(const MLX90640Frame &frame) noexcept;
	/**
	 * Extracts frame data's pattern (interleaved/chess).
	 * @param frame frame data
	 * @return frame data's pattern
	 */
	static MLX90640Pattern getPattern(const MLX90640Frame &frame) noexcept;
private:
	void readEEPROM(uint16_t *eeData);
	MLX90640Defect extractParameters(const uint16_t *eeData) noexcept;
	uint16_t busyWaitForData() const;
	bool readFrameData(uint16_t *frameData) const;
	float getVdd(const uint16_t *frameData) const noexcept;
	float getAmbientTemperature(const uint16_t *frameData,
			float vdd) const noexcept;
	void extractVDDParameters(const uint16_t *eeData) noexcept;
	void extractPTATParameters(const uint16_t *eeData) noexcept;
	void extractGainParameters(const uint16_t *eeData) noexcept;
	void extractTgcParameters(const uint16_t *eeData) noexcept;
	void extractResolutionParameters(const uint16_t *eeData) noexcept;
	void extractKsTaParameters(const uint16_t *eeData) noexcept;
	void extractKsToParameters(const uint16_t *eeData) noexcept;
	void extractAlphaParameters(const uint16_t *eeData) noexcept;
	void extractOffsetParameters(const uint16_t *eeData) noexcept;
	void extractKtaPixelParameters(const uint16_t *eeData) noexcept;
	void extractKvPixelParameters(const uint16_t *eeData) noexcept;
	void extractCPParameters(const uint16_t *eeData) noexcept;
	void extractCILCParameters(const uint16_t *eeData) noexcept;
	MLX90640Defect extractDeviatingPixels(const uint16_t *eeData) noexcept;
	bool checkAdjacentPixels(uint16_t pix1, uint16_t pix2) const noexcept;
	float getMedian(float *values, int n) const noexcept;
	bool isPixelBad(uint16_t pixel) const noexcept;
	bool validateFrameData(const uint16_t *frameData) const noexcept;
	bool validateAuxData(const uint16_t *auxData) const noexcept;
	void badPixelsCorrection(const uint16_t *pixels, MLX90640Pattern pattern,
			MLX90640Data &target) const noexcept;
public:
	/*! Default I2C address */
	static constexpr unsigned char I2C_ADDR = 0x33;
	/*! 32x24 resolution */
	static constexpr unsigned int PIXELS = 768;
	/*! Image rows count for 32x24 resolution */
	static constexpr unsigned int ROWS = 24;
	/*! Image columns count for 32x24 resolution */
	static constexpr unsigned int COLUMNS = 32;
	/*! Default emissivity value */
	static constexpr float DEFAULT_EMISSIVITY = 0.95;
	/*! Default reflected temperature value */
	static constexpr float DEFAULT_TR = 23.15;
	/*! Default open air shift in surrounding temperature */
	static constexpr float TA_SHIFT = 8;
	/*! Maximum wait time (milliseconds) after POR */
	static constexpr unsigned int POR_RESET_DELAY = 5000;
private:
	static constexpr unsigned int EEPROM_DUMP_COUNT = 832;
	static constexpr unsigned int AUX_DATA_COUNT = 64;
	struct {
		int16_t kVdd;
		int16_t vdd25;
		float KvPTAT;
		float KtPTAT;
		uint16_t vPTAT25;
		float alphaPTAT;
		int16_t gainEE;
		float tgc;
		float cpKv;
		float cpKta;
		uint8_t resolutionEE;
		uint8_t calibrationModeEE;
		float KsTa;
		float ksTo[5];
		int16_t ct[5];
		uint16_t alpha[768];
		uint8_t alphaScale;
		int16_t offset[768];
		int8_t kta[768];
		uint8_t ktaScale;
		int8_t kv[768];
		uint8_t kvScale;
		float cpAlpha[2];
		int16_t cpOffset[2];
		float ilChessC[3];
		uint16_t brokenPixels[5];
		uint16_t outlierPixels[5];
		MLX90640Defect defect;
	} params;
};

} /* namespace wanhive */

#endif /* WH_DRIVER_MLX90640_H_ */
