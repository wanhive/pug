/*
 * Image.h
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

#ifndef WH_VISION_IMAGE_H_
#define WH_VISION_IMAGE_H_

namespace wanhive {
/**
 * Image data structure.
 */
struct Image {
	/*! Image data */
	unsigned char *data;
	/*! Image size in bytes */
	unsigned long size;
	/*! Image height (rows) */
	unsigned int height;
	/*! Image width (columns) */
	unsigned int width;
};

/**
 * Raw image data structure.
 */
struct RawImage {
	/*! Raw image data */
	Image raw;
	/*! Raw image metadata */
	struct {
		/*! Buffer capacity in bytes */
		unsigned long bytes;
		/*! FOURCC code */
		unsigned long fourcc;
		/*! Image stride */
		unsigned int stride;
	} meta;
};

}  // namespace wanhive

#endif /* WH_VISION_IMAGE_H_ */
