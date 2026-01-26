/**
 * @file Jpeg.h
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

#ifndef WH_DEVICE_JPEG_H_
#define WH_DEVICE_JPEG_H_
#include "Image.h"

/*! @namespace wanhive */
namespace wanhive {
/**
 * Raw image to JPEG converter.
 * @note Supported input formats: uncompressed RGB data; YUV420; YUYV; MJPG.
 * @note Caller should not manage the output buffer.
 */
class Jpeg {
public:
	/**
	 * Constructor: sets up the jpeg converter.
	 * @param quality desired image quality [0-100]
	 */
	Jpeg(unsigned int quality = QUALITY) noexcept;
	~Jpeg();
	/**
	 * Returns the output image quality (%).
	 * @return current image quality
	 */
	unsigned int getQuality() const noexcept;
	/**
	 * Sets the output image quality [0-100].
	 * @param quality desired image quality
	 */
	void setQuality(unsigned int quality) noexcept;
	/**
	 * Converts raw image into a jpeg image.
	 * @param input raw image as input
	 * @param output jpeg image as output
	 */
	void convert(const RawImage &input, Image &output);
private:
	void fromRGB(const RawImage &input, Image &output);
	void fromYUYV(const RawImage &input, Image &output);
	void fromYUV420(const RawImage &input, Image &output);
	void fromMJPG(const RawImage &input, Image &output);
	void provision(unsigned long capacity);
public:
	/*! Default image quality */
	static constexpr unsigned int QUALITY = 90;
private:
	struct {
		unsigned char *data { };
		unsigned long capacity { };
		unsigned long used { };
		unsigned int quality { };
	} control;
};

} /* namespace wanhive */

#endif /* WH_DEVICE_JPEG_H_ */
