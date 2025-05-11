/*
 * ViewFinder.cpp
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

#include "ViewFinder.h"
#include <wanhive/base/common/Exception.h>
#include <wanhive/base/common/Memory.h>
#include <wanhive/base/unix/SystemException.h>
#include <sys/mman.h>
#include <cstring>

namespace wanhive {

ViewFinder::ViewFinder() {
	setup(0, 0);
}

ViewFinder::ViewFinder(unsigned int height, unsigned int width) {
	setup(height, width);
}

ViewFinder::~ViewFinder() {
	clear();
}

bool ViewFinder::capture(RawImage &image) {
	try {
		libcamera::Request *request = nullptr;
		if (queue.get(request)) {
			auto status = _capture(request, image);
			reuse(request);
			return status;
		} else {
			return false;
		}

	} catch (...) {
		throw Exception(EX_OPERATION);
	}
}

bool ViewFinder::capture(Image &image) {
	try {
		libcamera::Request *request = nullptr;
		if (queue.get(request)) {
			auto status = _capture(request, image);
			reuse(request);
			return status;
		} else {
			return false;
		}
	} catch (...) {
		throw Exception(EX_OPERATION);
	}
}

bool ViewFinder::capture(Image &image, unsigned int quality) {
	try {
		libcamera::Request *request = nullptr;
		if (queue.get(request)) {
			auto status = _capture(request, image, quality);
			reuse(request);
			return status;
		} else {
			return false;
		}
	} catch (...) {
		throw Exception(EX_OPERATION);
	}
}

bool ViewFinder::_capture(libcamera::Request *request, RawImage &image) noexcept {
	try {
		/* Copy the mapped data */
		const auto &data = mapped[request];
		provision(data.second);
		memcpy(captured.raw.data, data.first, data.second);
		/* Configure the output */
		captured.raw.size = 0;
		//-----------------------------------------------------------------
		auto &buffers = request->buffers();
		for (auto &buffer : buffers) {
			auto &cfg = buffer.first->configuration();
			auto &metadata = buffer.second->metadata();
			captured.raw.height = cfg.size.height;
			captured.raw.width = cfg.size.width;
			captured.meta.fourcc = cfg.pixelFormat.fourcc();
			captured.meta.stride = cfg.stride;
			for (auto &plane : metadata.planes()) {
				captured.raw.size += plane.bytesused;
			}
		}
		//-----------------------------------------------------------------
		image = captured;
		return true;
	} catch (...) {
		return false;
	}
}

bool ViewFinder::_capture(libcamera::Request *request, Image &image) noexcept {
	try {
		RawImage cap;
		const auto &raw = mapped[request];
		cap.raw.data = static_cast<unsigned char*>(raw.first);
		cap.meta.bytes = raw.second;
		cap.raw.size = 0;
		//-----------------------------------------------------------------
		auto &buffers = request->buffers();
		for (auto &buffer : buffers) {
			auto &cfg = buffer.first->configuration();
			auto &metadata = buffer.second->metadata();
			cap.meta.stride = cfg.stride;
			cap.meta.fourcc = cfg.pixelFormat.fourcc();
			cap.raw.width = cfg.size.width;
			cap.raw.height = cfg.size.height;
			for (auto &plane : metadata.planes()) {
				cap.raw.size += plane.bytesused;
			}
		}
		//-----------------------------------------------------------------
		jpeg.convert(cap, image);
		return true;
	} catch (...) {
		return false;
	}
}

bool ViewFinder::_capture(libcamera::Request *request, Image &image,
		unsigned int quality) noexcept {
	jpeg.setQuality(quality);
	return _capture(request, image);
}

void ViewFinder::setup(unsigned int height, unsigned int width) {
	try {
		memset(&captured, 0, sizeof(captured));
		acquire();
		configure(height, width);
		allocate();
	} catch (const BaseException &e) {
		clear();
		throw;
	} catch (...) {
		clear();
		throw Exception(EX_RESOURCE);
	}
}

void ViewFinder::acquire() {
	cm = std::make_unique<libcamera::CameraManager>();
	cm->start();
	auto cameras = cm->cameras();
	if (cameras.empty()) {
		throw Exception(EX_RESOURCE);
	}

	camera = cm->get(cameras[0]->id());
	camera->acquire();
}

void ViewFinder::configure(unsigned int height, unsigned int width) {
	config = camera->generateConfiguration(
			{ libcamera::StreamRole::Viewfinder });
	auto &scfg = config->at(0);
	if (height && width) {
		scfg.size.height = height;
		scfg.size.width = width;
	}
	scfg.pixelFormat = libcamera::formats::YUV420;
	scfg.bufferCount = 1;
	scfg.stride = 0;
	if (config->validate() != libcamera::CameraConfiguration::Invalid) {
		camera->configure(config.get());
	} else {
		throw Exception(EX_PARAMETER);
	}
}

void ViewFinder::allocate() {
	allocator = std::make_unique<libcamera::FrameBufferAllocator>(camera);
	for (auto &scfg : *config) {
		if (allocator->allocate(scfg.stream()) < 0) {
			throw Exception(EX_MEMORY);
		}

		auto stream = scfg.stream();
		auto &buffers = allocator->buffers(stream);
		//-----------------------------------------------------------------
		/* Queue up the requests */
		for (auto &buffer : buffers) {
			auto request = camera->createRequest();
			if (!request) {
				throw Exception(EX_MEMORY);
			}

			if (request->addBuffer(stream, buffer.get()) < 0) {
				throw Exception(EX_OPERATION);
			}
			//-----------------------------------------------------------------
			/* Allocate image data memory and create a memory map */
			unsigned long bytes = 0;
			auto &firstPlane = buffer->planes()[0];
			for (const auto &plane : buffer->planes()) {
				bytes += plane.length;
			}

			auto buf = mmap(nullptr, bytes, PROT_READ, MAP_SHARED,
					firstPlane.fd.get(), 0);

			if (!buf) {
				throw SystemException();
			} else {
				mapped[request.get()] = { buf, bytes };
			}
			//-----------------------------------------------------------------
			/* Put into the records */
			requests.push_back(std::move(request));
		}
	}
	//-----------------------------------------------------------------
	queue.initialize(requests.size() + 1);
	camera->requestCompleted.connect(this, &ViewFinder::process);
	camera->start();
	for (auto &request : requests) {
		camera->queueRequest(request.get());
	}
}

void ViewFinder::process(libcamera::Request *request) {
	if (request->status() == libcamera::Request::RequestCancelled) {
		return;
	}

	queue.put(request);
}

void ViewFinder::reuse(libcamera::Request *request) {
	request->reuse(libcamera::Request::ReuseBuffers);
	camera->queueRequest(request);
}

void ViewFinder::clear() noexcept {
	if (camera.get()) {
		camera->stop();
		camera->requestCompleted.disconnect(this, &ViewFinder::process);
		queue.clear();
		for (auto &itr : mapped) {
			munmap(itr.second.first, itr.second.second);
		}

		mapped.clear();
		requests.clear();
		if (allocator.get() && config.get()) {
			allocator->free(config->at(0).stream());
		}

		allocator.reset();
		config.reset();
		camera->release();
		camera.reset();
	}

	if (cm.get()) {
		cm->stop();
		cm.reset();
	}

	Memory<unsigned char>::free(captured.raw.data);
	memset(&captured, 0, sizeof(captured));
}

void ViewFinder::provision(unsigned long capacity) {
	if (capacity && (captured.meta.bytes < capacity)) {
		Memory<unsigned char, false>::resize(captured.raw.data, capacity);
		captured.meta.bytes = capacity;
	}
}

} /* namespace wanhive */
