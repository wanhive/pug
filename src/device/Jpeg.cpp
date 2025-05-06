/*
 * Jpeg.cpp
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
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 */

#include "Jpeg.h"
#include <wanhive/base/common/Exception.h>
#include <wanhive/base/common/Memory.h>
#include <algorithm>
#include <cstdlib>
#include <cstdint>
#include <cstdio>
#include <cstring>
extern "C" {
#include <jpeglib.h>
}

namespace {

enum WHFOURCC : unsigned long {
	WH_YUV420 = 842093913, WH_YUYV = 1448695129, WH_MJPG = 1196444237
};

}  // namespace
namespace wanhive {

Jpeg::Jpeg(unsigned int quality) noexcept {
	setQuality(quality);
}

Jpeg::~Jpeg() {
	Memory<unsigned char>::free(control.data);
}

unsigned int Jpeg::getQuality() const noexcept {
	return control.quality;
}

void Jpeg::setQuality(unsigned int quality) noexcept {
	if (quality < 10) {
		control.quality = 10;
	} else if (quality > 100) {
		control.quality = 100;
	} else {
		control.quality = quality;
	}
}

void Jpeg::convert(const RawImage &input, Image &output) {
	if (!input.raw.data || !input.meta.bytes || !input.raw.size
			|| !input.raw.height || !input.raw.width
			|| (input.meta.bytes < input.raw.size)) {
		throw Exception(EX_ARGUMENT);
	}

	memset(&output, 0, sizeof(output));
	switch (input.meta.fourcc) {
	case WH_YUV420: //YUV420
		fromYUV420(input, output);
		break;
	case WH_YUYV: //YUYV
		fromYUYV(input, output);
		break;
	case WH_MJPG: //MJPG
		fromMJPG(input, output);
		break;
	default:
		throw Exception(EX_ARGUMENT);
	}
}

void Jpeg::fromYUYV(const RawImage &input, Image &output) {
	provision(input.meta.bytes);
	const unsigned int output_width3 = 3 * input.raw.width;
	auto tmp_row = Memory<uint8_t>::allocate(output_width3);
	if (!tmp_row) {
		throw Exception(EX_MEMORY);
	}
	auto h_offset = Memory<unsigned int>::allocate(output_width3);
	if (!h_offset) {
		Memory<uint8_t>::free(tmp_row);
		throw Exception(EX_MEMORY);
	}

	// Pre-calculate the horizontal offsets to speed up the main loop.
	for (unsigned int i = 0, k = 0; i < input.raw.width; i++) {
		unsigned int off = (i * input.raw.width) / input.raw.width * 2;
		unsigned int off_align = off & ~3;
		h_offset[k++] = off;
		h_offset[k++] = off_align + 1;
		h_offset[k++] = off_align + 3;
	}

	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;

	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&cinfo);

	cinfo.image_width = input.raw.width;
	cinfo.image_height = input.raw.height;
	cinfo.input_components = 3;
	cinfo.in_color_space = JCS_YCbCr;
	cinfo.restart_interval = 0;

	jpeg_set_defaults(&cinfo);
	jpeg_set_quality(&cinfo, control.quality, TRUE);
	control.used = control.capacity;
	jpeg_mem_dest(&cinfo, &control.data, &control.used);
	jpeg_start_compress(&cinfo, TRUE);

	JSAMPROW jrow[1];
	jrow[0] = &tmp_row[0];
	while (cinfo.next_scanline < input.raw.height) {
		unsigned int offset = ((cinfo.next_scanline * input.raw.height)
				/ input.raw.height) * input.meta.stride;
		for (unsigned int k = 0; k < output_width3; k += 3) {
			tmp_row[k] = input.raw.data[offset + h_offset[k]];
			tmp_row[k + 1] = input.raw.data[offset + h_offset[k + 1]];
			tmp_row[k + 2] = input.raw.data[offset + h_offset[k + 2]];
		}
		jpeg_write_scanlines(&cinfo, jrow, 1);
	}

	jpeg_finish_compress(&cinfo);
	jpeg_destroy_compress(&cinfo);
	Memory<uint8_t>::free(tmp_row);
	Memory<unsigned int>::free(h_offset);

	output.data = control.data;
	output.size = control.used;
	output.height = input.raw.height;
	output.width = input.raw.width;
}

void Jpeg::fromYUV420(const RawImage &input, Image &output) {
	provision(input.meta.bytes);
	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;

	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&cinfo);

	cinfo.image_width = input.raw.width;
	cinfo.image_height = input.raw.height;
	cinfo.input_components = 3;
	cinfo.in_color_space = JCS_YCbCr;
	cinfo.restart_interval = 0;

	jpeg_set_defaults(&cinfo);
	cinfo.raw_data_in = TRUE;
	jpeg_set_quality(&cinfo, control.quality, TRUE);
	control.used = control.capacity;
	jpeg_mem_dest(&cinfo, &control.data, &control.used);
	jpeg_start_compress(&cinfo, TRUE);

	int stride2 = input.meta.stride / 2;
	uint8_t *Y = (uint8_t*) input.raw.data;
	uint8_t *U = (uint8_t*) Y + input.meta.stride * input.raw.height;
	uint8_t *V = (uint8_t*) U + stride2 * (input.raw.height / 2);
	uint8_t *Y_max = U - input.meta.stride;
	uint8_t *U_max = V - stride2;
	uint8_t *V_max = U_max + stride2 * (input.raw.height / 2);

	JSAMPROW y_rows[16];
	JSAMPROW u_rows[8];
	JSAMPROW v_rows[8];

	for (uint8_t *Y_row = Y, *U_row = U, *V_row = V;
			cinfo.next_scanline < input.raw.height;) {
		for (int i = 0; i < 16; i++, Y_row += input.meta.stride)
			y_rows[i] = std::min(Y_row, Y_max);
		for (int i = 0; i < 8; i++, U_row += stride2, V_row += stride2)
			u_rows[i] = std::min(U_row, U_max), v_rows[i] = std::min(V_row,
					V_max);

		JSAMPARRAY rows[] = { y_rows, u_rows, v_rows };
		jpeg_write_raw_data(&cinfo, rows, 16);
	}

	jpeg_finish_compress(&cinfo);
	jpeg_destroy_compress(&cinfo);
	output.data = control.data;
	output.size = control.used;
	output.height = input.raw.height;
	output.width = input.raw.width;
}

void Jpeg::fromMJPG(const RawImage &input, Image &output) {
	provision(input.meta.bytes);
	control.used = input.raw.size;
	memcpy(control.data, input.raw.data, control.used);

	output.data = control.data;
	output.size = control.used;
	output.height = input.raw.height;
	output.width = input.raw.width;
}

void Jpeg::provision(unsigned long capacity) {
	if (capacity && (control.capacity < capacity)) {
		Memory<unsigned char, false>::resize(control.data, capacity);
		control.capacity = capacity;
	}
}

} /* namespace wanhive */
