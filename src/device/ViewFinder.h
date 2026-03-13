/**
 * @file ViewFinder.h
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

#ifndef WH_DEVICE_VIEWFINDER_H_
#define WH_DEVICE_VIEWFINDER_H_
#include "Jpeg.h"
#include <libcamera/libcamera.h>
#include <wanhive/base/ds/CircularBuffer.h>
#include <map>
#include <memory>
#include <vector>

/*! @namespace wanhive */
namespace wanhive {
/**
 * A bare-minimum USB/MIPI camera driver.
 * @ref https://libcamera.org/guides/application-developer.html
 */
class ViewFinder {
public:
	/**
	 * Constructor: configures a camera.
	 */
	ViewFinder();
	/**
	 * Constructor: configures a camera.
	 * @param height number of rows (pixels)
	 * @param width number of columns (pixels)
	 */
	ViewFinder(unsigned int height, unsigned int width);
	/**
	 * Destructor: releases the camera.
	 */
	~ViewFinder();
	/**
	 * Captures raw image from a camera.
	 * @param image raw image output
	 * @return true on success, false on failure (new data not available)
	 */
	bool capture(RawImage &image);
	/**
	 * Captures JPEG image from a camera.
	 * @param image JPEG image output
	 * @return true on success, false on failure (new data not available)
	 */
	bool capture(Image &image);
	/**
	 * Captures JPEG image from a camera.
	 * @param image JPEG image output
	 * @param quality desired image quality [0-100]
	 * @return true on success, false on failure (new data not available)
	 */
	bool capture(Image &image, unsigned int quality);
private:
	bool capture(libcamera::Request *request, RawImage &image) noexcept;
	bool capture(libcamera::Request *request, Image &image) noexcept;
	bool capture(libcamera::Request *request, Image &image,
			unsigned int quality) noexcept;
	void setup(unsigned int height, unsigned int width);
	void acquire();
	void configure(unsigned int height, unsigned int width);
	void allocate();
	void process(libcamera::Request *request);
	void reuse(libcamera::Request *request);
	void clear() noexcept;
	void allocate(libcamera::Request *request, int fd, unsigned long bytes);
	void provision(unsigned long capacity);
private:
	std::unique_ptr<libcamera::CameraManager> cm;
	std::shared_ptr<libcamera::Camera> camera;
	std::unique_ptr<libcamera::CameraConfiguration> config;
	std::unique_ptr<libcamera::FrameBufferAllocator> allocator;
	std::vector<std::unique_ptr<libcamera::Request>> requests;
	std::map<libcamera::Request*, std::pair<void*, unsigned long>> mapped;
	CircularBuffer<libcamera::Request*, true> queue;
	RawImage captured;
	Jpeg jpeg;
};

} /* namespace wanhive */

#endif /* WH_DEVICE_VIEWFINDER_H_ */
