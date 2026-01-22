/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * dng_encoder.cpp - DNG video encoder.
 */

#include <chrono>
#include <cstring>
#include <ctime>
#include <limits>
#include <map>
#include <vector>
#include <cmath>

#include <tiffio.h>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>

#include "dng_encoder.hpp"
#include "core/logging.hpp"
#include "core/metadata.hpp"
#include <libcamera/controls.h>

#ifndef MAKE_STRING
#define MAKE_STRING "Wassoc"
#endif

using namespace libcamera;

static char TIFF_RGGB[4] = { 0, 1, 1, 2 };
static char TIFF_GRBG[4] = { 1, 0, 2, 1 };
static char TIFF_BGGR[4] = { 2, 1, 1, 0 };
static char TIFF_GBRG[4] = { 1, 2, 0, 1 };
static char TIFF_MONO[4] = { 0, 0, 0, 0 };

struct BayerFormat
{
	char const *name;
	int bits;
	char const *order;
	bool packed;
	bool compressed;
};

static const std::map<PixelFormat, BayerFormat> bayer_formats =
{
	{ formats::SGRBG8, { "GRBG-8", 8, TIFF_GRBG, false, false } },

	{ formats::SRGGB10_CSI2P, { "RGGB-10", 10, TIFF_RGGB, true, false } },
	{ formats::SGRBG10_CSI2P, { "GRBG-10", 10, TIFF_GRBG, true, false } },
	{ formats::SBGGR10_CSI2P, { "BGGR-10", 10, TIFF_BGGR, true, false } },
	{ formats::SGBRG10_CSI2P, { "GBRG-10", 10, TIFF_GBRG, true, false } },

	{ formats::SRGGB10, { "RGGB-10", 10, TIFF_RGGB, false, false } },
	{ formats::SGRBG10, { "GRBG-10", 10, TIFF_GRBG, false, false } },
	{ formats::SBGGR10, { "BGGR-10", 10, TIFF_BGGR, false, false } },
	{ formats::SGBRG10, { "GBRG-10", 10, TIFF_GBRG, false, false } },

	{ formats::SRGGB12_CSI2P, { "RGGB-12", 12, TIFF_RGGB, true, false } },
	{ formats::SGRBG12_CSI2P, { "GRBG-12", 12, TIFF_GRBG, true, false } },
	{ formats::SBGGR12_CSI2P, { "BGGR-12", 12, TIFF_BGGR, true, false } },
	{ formats::SGBRG12_CSI2P, { "GBRG-12", 12, TIFF_GBRG, true, false } },

	{ formats::SRGGB12, { "RGGB-12", 12, TIFF_RGGB, false, false } },
	{ formats::SGRBG12, { "GRBG-12", 12, TIFF_GRBG, false, false } },
	{ formats::SBGGR12, { "BGGR-12", 12, TIFF_BGGR, false, false } },
	{ formats::SGBRG12, { "GBRG-12", 12, TIFF_GBRG, false, false } },

	{ formats::SRGGB16, { "RGGB-16", 16, TIFF_RGGB, false, false } },
	{ formats::SGRBG16, { "GRBG-16", 16, TIFF_GRBG, false, false } },
	{ formats::SBGGR16, { "BGGR-16", 16, TIFF_BGGR, false, false } },
	{ formats::SGBRG16, { "GBRG-16", 16, TIFF_GBRG, false, false } },

	{ formats::R10_CSI2P, { "BGGR-10", 10, TIFF_BGGR, true, false } },
	{ formats::R10, { "BGGR-10", 10, TIFF_BGGR, false, false } },
	{ formats::R12, { "BGGR-12", 12, TIFF_BGGR, false, false } },

	/* PiSP compressed formats. */
	{ formats::RGGB_PISP_COMP1, { "RGGB-16-PISP", 16, TIFF_RGGB, false, true } },
	{ formats::GRBG_PISP_COMP1, { "GRBG-16-PISP", 16, TIFF_GRBG, false, true } },
	{ formats::GBRG_PISP_COMP1, { "GBRG-16-PISP", 16, TIFF_GBRG, false, true } },
	{ formats::BGGR_PISP_COMP1, { "BGGR-16-PISP", 16, TIFF_BGGR, false, true } },
};

// Helper functions from dng.cpp
static void unpack_10bit(uint8_t const *src, StreamInfo const &info, uint8_t *dest, uint16_t *dest16Bit)
{
	unsigned int w_align = info.width & ~3;
	for (unsigned int y = 0; y < info.height; y++, src += info.stride)
	{
		uint8_t const *ptr = src;
		unsigned int x;
		for (x = 0; x < w_align; x += 4, ptr += 5)
		{
			uint16_t val1 = (ptr[0] << 2) | ((ptr[4] >> 0) & 3);
			uint16_t val2 = (ptr[1] << 2) | ((ptr[4] >> 2) & 3);
			uint16_t val3 = (ptr[2] << 2) | ((ptr[4] >> 4) & 3);
			uint16_t val4 = (ptr[3] << 2) | ((ptr[4] >> 6) & 3);
			uint8_t byte1 = val1 >> 2;
			uint8_t byte2 = ((val1 & 3) << 6) | (val2 >> 4);
			uint8_t byte3 = ((val2 & 0xf) << 4) | (val3 >> 6);
			uint8_t byte4 = ((val3 & 0x3f) << 2) | (val4 >> 8);
			uint8_t byte5 = val4 & 0xff;
			*dest++ = byte1;
			*dest++ = byte2;
			*dest++ = byte3;
			*dest++ = byte4;
			*dest++ = byte5;
			*dest16Bit++ = val1;
			*dest16Bit++ = val2;
			*dest16Bit++ = val3;
			*dest16Bit++ = val4;
		}
	}
}

static void unpack_12bit(uint8_t const *src, StreamInfo const &info, uint8_t *dest, uint16_t *dest16Bit)
{
	unsigned int w_align = info.width & ~1;
	for (unsigned int y = 0; y < info.height; y++, src += info.stride)
	{
		uint8_t const *ptr = src;
		unsigned int x;
		for (x = 0; x < w_align; x += 2, ptr += 3)
		{
			uint16_t val1 = (ptr[0] << 4) | ((ptr[2] >> 0) & 15);
			uint16_t val2 = (ptr[1] << 4) | ((ptr[2] >> 4) & 15);
			uint8_t byte1 = val1 >> 4;
			uint8_t byte2 = ((val1 & 0xf) << 4) | (val2 >> 8);
			uint8_t byte3 = val2 & 0xff;
			*dest++ = byte1;
			*dest++ = byte2;
			*dest++ = byte3;
			*dest16Bit++ = val1;
			*dest16Bit++ = val2;
		}
	}
}

static void unpack_12bit_to_8bit(uint8_t const *src, StreamInfo const &info, uint8_t *dest, uint16_t *dest16Bit)
{
	unsigned int w_align = info.width & ~1;
	for (unsigned int y = 0; y < info.height; y++, src += info.stride)
	{
		uint8_t const *ptr = src;
		unsigned int x;
		for (x = 0; x < w_align; x += 2, ptr += 3)
		{
			uint16_t val1 = (ptr[0] << 4) | ((ptr[2] >> 0) & 15);
			uint16_t val2 = (ptr[1] << 4) | ((ptr[2] >> 4) & 15);
			uint8_t val1_as_8bit = ((float)val1 / 4096.f) * 256;
			uint8_t val2_as_8bit = ((float)val2 / 4096.f) * 256;
			*dest++ = val1_as_8bit;
			*dest++ = val2_as_8bit;
			*dest16Bit++ = val1;
			*dest16Bit++ = val2;
		}
	}
}

static void unpack_12bit_to_10bit(uint8_t const *src, StreamInfo const &info, uint8_t *dest, uint16_t *dest16Bit)
{
	unsigned int w_align = info.width & ~1;
	for (unsigned int y = 0; y < info.height; y++, src += info.stride)
	{
		uint8_t const *ptr = src;
		unsigned int x;
		for (x = 0; x < w_align; x += 4, ptr += 6)
		{
			uint16_t val1 = (ptr[0] << 4) | ((ptr[2] >> 0) & 15);
			uint16_t val2 = (ptr[1] << 4) | ((ptr[2] >> 4) & 15);
			uint16_t val3 = (ptr[3] << 4) | ((ptr[5] >> 0) & 15);
			uint16_t val4 = (ptr[4] << 4) | ((ptr[5] >> 4) & 15);
			uint16_t val1_as_10bit = ((float)val1 / 4096.f) * 1024;
			uint16_t val2_as_10bit = ((float)val2 / 4096.f) * 1024;
			uint16_t val3_as_10bit = ((float)val3 / 4096.f) * 1024;
			uint16_t val4_as_10bit = ((float)val4 / 4096.f) * 1024;
			uint8_t byte1 = val1_as_10bit >> 2;
			uint8_t byte2 = ((val1_as_10bit & 3) << 6) | (val2_as_10bit >> 4);
			uint8_t byte3 = ((val2_as_10bit & 0xf) << 4) | (val3_as_10bit >> 6);
			uint8_t byte4 = ((val3_as_10bit & 0x3f) << 2) | (val4_as_10bit >> 8);
			uint8_t byte5 = val4_as_10bit & 0xff;
			*dest++ = byte1;
			*dest++ = byte2;
			*dest++ = byte3;
			*dest++ = byte4;
			*dest++ = byte5;
			*dest16Bit++ = val1;
			*dest16Bit++ = val2;
			*dest16Bit++ = val3;
			*dest16Bit++ = val4;
		}
	}
}

static void unpack_16bit(uint8_t const *src, StreamInfo const &info, uint16_t *dest)
{
	unsigned int w = info.width;
	for (unsigned int y = 0; y < info.height; y++)
	{
		memcpy(dest, src, 2 * w);
		dest += w;
		src += info.stride;
	}
}

static void copy_8bit(uint8_t const *src, StreamInfo const &info, uint8_t *dest, uint16_t *dest16Bit)
{
	unsigned int w = info.width;
	for (unsigned int y = 0; y < info.height; y++)
	{
		memcpy(dest, src, w);
		for (unsigned int x = 0; x < w; x++)
			dest16Bit[x] = src[x];
		dest += w;
		dest16Bit += w;
		src += info.stride;
	}
}

static void uncompress(uint8_t const *src, StreamInfo const &info, uint16_t *dest)
{
	unsigned int buf_stride_pixels = (info.width + 7) & ~7;
	for (unsigned int y = 0; y < info.height; ++y)
	{
		uint16_t *dp = dest + y * buf_stride_pixels;
		uint8_t const *sp = src + y * info.stride;
		for (unsigned int x = 0; x < info.width; x+=8)
		{
			if (COMPRESS_MODE & 1)
			{
				uint32_t w0 = 0, w1 = 0;
				for (int b = 0; b < 4; ++b)
					w0 |= (*sp++) << (b * 8);
				for (int b = 0; b < 4; ++b)
					w1 |= (*sp++) << (b * 8);
				subBlockFunction(dp, w0);
				subBlockFunction(dp + 1, w1);
				for (int i = 0; i < 8; ++i, ++dp)
					*dp = postprocess(*dp);
			}
			else
			{
				for (int i = 0; i < 8; ++i)
					*dp++ = postprocess((*sp++) << 8);
			}
		}
	}
}

// Matrix class (from dng.cpp)
struct Matrix
{
	Matrix(float m0, float m1, float m2,
		   float m3, float m4, float m5,
		   float m6, float m7, float m8)
	{
		m[0] = m0, m[1] = m1, m[2] = m2;
		m[3] = m3, m[4] = m4, m[5] = m5;
		m[6] = m6, m[7] = m7, m[8] = m8;
	}
	Matrix(float diag0, float diag1, float diag2) : Matrix(diag0, 0, 0, 0, diag1, 0, 0, 0, diag2) {}
	Matrix() {}
	float m[9];
	Matrix T() const
	{
		return Matrix(m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]);
	}
	Matrix C() const
	{
		return Matrix(m[4] * m[8] - m[5] * m[7], -(m[3] * m[8] - m[5] * m[6]), m[3] * m[7] - m[4] * m[6],
					  -(m[1] * m[8] - m[2] * m[7]), m[0] * m[8] - m[2] * m[6], -(m[0] * m[7] - m[1] * m[6]),
					  m[1] * m[5] - m[2] * m[4], -(m[0] * m[5] - m[2] * m[3]), m[0] * m[4] - m[1] * m[3]);
	}
	Matrix Adj() const { return C().T(); }
	float Det() const
	{
		return (m[0] * (m[4] * m[8] - m[5] * m[7]) -
				m[1] * (m[3] * m[8] - m[5] * m[6]) +
				m[2] * (m[3] * m[7] - m[4] * m[6]));
	}
	Matrix Inv() const { return Adj() * (1.0 / Det()); }
	Matrix operator*(Matrix const &other) const
	{
		Matrix result;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				result.m[i * 3 + j] =
					m[i * 3] * other.m[j] + m[i * 3 + 1] * other.m[3 + j] + m[i * 3 + 2] * other.m[6 + j];
		return result;
	}
	Matrix operator*(float const &f) const
	{
		Matrix result;
		for (int i = 0; i < 9; i++)
			result.m[i] = m[i] * f;
		return result;
	}
};

// Structure to hold memory buffer for TIFF encoding
struct TiffMemoryBuffer
{
	uint8_t *data;
	size_t size;
	size_t capacity;
	size_t position;
};

// Custom read function for libtiff (not used for writing, but required)
static tsize_t tiff_read(thandle_t handle, tdata_t data, tsize_t size)
{
	(void)handle;
	(void)data;
	(void)size;
	return 0;
}

// Custom write function for libtiff to write to memory
static tsize_t tiff_write(thandle_t handle, tdata_t data, tsize_t size)
{
	TiffMemoryBuffer *buffer = (TiffMemoryBuffer *)handle;
	
	// Grow buffer if needed
	if (buffer->position + size > buffer->capacity)
	{
		size_t new_capacity = buffer->capacity * 2;
		if (new_capacity < buffer->position + size)
			new_capacity = buffer->position + size + 1024 * 1024; // Add 1MB extra space
		
		uint8_t *new_data = (uint8_t *)realloc(buffer->data, new_capacity);
		if (!new_data)
			return -1;
		buffer->data = new_data;
		buffer->capacity = new_capacity;
	}
	
	memcpy(buffer->data + buffer->position, data, size);
	buffer->position += size;
	if (buffer->position > buffer->size)
		buffer->size = buffer->position;
	
	return size;
}

// Custom seek function for libtiff
static toff_t tiff_seek(thandle_t handle, toff_t offset, int whence)
{
	TiffMemoryBuffer *buffer = (TiffMemoryBuffer *)handle;
	
	switch (whence)
	{
	case SEEK_SET:
		buffer->position = offset;
		break;
	case SEEK_CUR:
		buffer->position += offset;
		break;
	case SEEK_END:
		buffer->position = buffer->size + offset;
		break;
	default:
		return -1;
	}
	
	// Grow buffer if seeking beyond current size
	if (buffer->position > buffer->capacity)
	{
		size_t new_capacity = buffer->position + 1024 * 1024;
		uint8_t *new_data = (uint8_t *)realloc(buffer->data, new_capacity);
		if (!new_data)
			return -1;
		buffer->data = new_data;
		buffer->capacity = new_capacity;
	}
	
	return buffer->position;
}

// Custom close function for libtiff (no-op for memory)
static int tiff_close(thandle_t handle)
{
	(void)handle;
	return 0;
}

// Custom size function for libtiff
static toff_t tiff_size(thandle_t handle)
{
	TiffMemoryBuffer *buffer = (TiffMemoryBuffer *)handle;
	return buffer->size;
}

// Custom map function for libtiff (not used, but required)
static int tiff_map(thandle_t handle, tdata_t *base, toff_t *size)
{
	(void)handle;
	(void)base;
	(void)size;
	return 0;
}

// Custom unmap function for libtiff (not used, but required)
static void tiff_unmap(thandle_t handle, tdata_t base, toff_t size)
{
	(void)handle;
	(void)base;
	(void)size;
}

// Include helper functions from dng.cpp - we'll need to copy them or include them
// For now, let's include the necessary parts inline or reference the original file
// We'll need: unpack_10bit, unpack_12bit, unpack_12bit_to_8bit, unpack_12bit_to_10bit, 
// unpack_16bit, copy_8bit, uncompress, and compression helpers

// Compression helpers (from dng.cpp)
#define COMPRESS_OFFSET 2048
#define COMPRESS_MODE 1

static uint16_t postprocess(uint16_t a)
{
	if (COMPRESS_MODE & 2)
	{
		if (COMPRESS_MODE == 3 && a < 0x4000)
			a = a >> 2;
		else if (a < 0x1000)
			a = a >> 4;
		else if (a < 0x1800)
			a = (a - 0x800) >> 3;
		else if (a < 0x3000)
			a = (a - 0x1000) >> 2;
		else if (a < 0x6000)
			a = (a - 0x2000) >> 1;
		else if (a < 0xC000)
			a = (a - 0x4000);
		else
			a = 2 * (a - 0x8000);
	}
	return std::min(0xFFFF, a + COMPRESS_OFFSET);
}

static uint16_t dequantize(uint16_t q, int qmode)
{
	switch (qmode)
	{
	case 0:
		return (q < 320) ? 16 * q : 32 * (q - 160);
	case 1:
		return 64 * q;
	case 2:
		return 128 * q;
	default:
		return (q < 94) ? 256 * q : std::min(0xFFFF, 512 * (q - 47));
	}
}

static void subBlockFunction(uint16_t *d, uint32_t w)
{
	int q[4];
	int qmode = (w & 3);
	if (qmode < 3)
	{
		int field0 = (w >> 2) & 511;
		int field1 = (w >> 11) & 127;
		int field2 = (w >> 18) & 127;
		int field3 = (w >> 25) & 127;
		if (qmode == 2 && field0 >= 384)
		{
			q[1] = field0;
			q[2] = field1 + 384;
		}
		else
		{
			q[1] = (field1 >= 64) ? field0 : field0 + 64 - field1;
			q[2] = (field1 >= 64) ? field0 + field1 - 64 : field0;
		}
		int p1 = std::max(0, q[1] - 64);
		if (qmode == 2)
			p1 = std::min(384, p1);
		int p2 = std::max(0, q[2] - 64);
		if (qmode == 2)
			p2 = std::min(384, p2);
		q[0] = p1 + field2;
		q[3] = p2 + field3;
	}
	else
	{
		int pack0 = (w >> 2) & 32767;
		int pack1 = (w >> 17) & 32767;
		q[0] = (pack0 & 15) + 16 * ((pack0 >> 8) / 11);
		q[1] = (pack0 >> 4) % 176;
		q[2] = (pack1 & 15) + 16 * ((pack1 >> 8) / 11);
		q[3] = (pack1 >> 4) % 176;
	}
	d[0] = dequantize(q[0], qmode);
	d[2] = dequantize(q[1], qmode);
	d[4] = dequantize(q[2], qmode);
	d[6] = dequantize(q[3], qmode);
}

// Helper functions from dng.cpp - we need to include them
// For brevity, I'll reference that these should be copied from dng.cpp
// In a real implementation, you might want to extract these to a shared header

// Note: The unpack functions and uncompress function need to be included here
// For now, I'll create a simplified version that references the original functions
// In practice, you'd want to either:
// 1. Extract these to a shared header/implementation file
// 2. Include them directly here
// 3. Link against the dng.cpp object file

// For this implementation, I'll create a wrapper that can call the original dng_save logic
// but adapted for memory output. Actually, let me create a complete implementation
// that includes all the necessary helper functions inline.

// Since the file is getting long, let me create the encoder structure first,
// then we can refactor to share code with dng.cpp if needed.

DngEncoder::DngEncoder(VideoOptions const *options)
	: Encoder(options), abortEncode_(false), abortOutput_(false), index_(0)
{
	options_ = options;
	output_thread_ = std::thread(&DngEncoder::outputThread, this);
	for (int i = 0; i < NUM_ENC_THREADS; i++)
		encode_thread_[i] = std::thread(std::bind(&DngEncoder::encodeThread, this, i));
	LOG(2, "Opened DngEncoder");
}

DngEncoder::~DngEncoder()
{
	abortEncode_ = true;
	for (int i = 0; i < NUM_ENC_THREADS; i++)
		encode_thread_[i].join();
	abortOutput_ = true;
	output_thread_.join();
	LOG(2, "DngEncoder closed");
}

void DngEncoder::EncodeBuffer(int fd, size_t size, void *mem, StreamInfo const &info, int64_t timestamp_us, Metadata const &post_process_metadata, libcamera::ControlList const &control_list_metadata)
{
	std::lock_guard<std::mutex> lock(encode_mutex_);
	EncodeItem item = { mem, info, timestamp_us, index_++, post_process_metadata, control_list_metadata };
	encode_queue_.push(item);
	encode_cond_var_.notify_all();
}

// This is a large function - we'll need to adapt the dng_save logic
// For now, let me create a structure that calls the core DNG writing logic
// but outputs to memory instead of a file

void DngEncoder::encodeDNG(EncodeItem &item, uint8_t *&encoded_buffer, size_t &buffer_len)
{
	LOG(1, "Encoding DNG to memory buffer");
	LOG(1, "Pixel format: " << item.info.pixel_format.toString());
	
	// Check the Bayer format
	auto it = bayer_formats.find(item.info.pixel_format);
	if (it == bayer_formats.end())
		throw std::runtime_error("unsupported Bayer format");
	
	BayerFormat const &bayer_format = it->second;
	bool force8bit = options_->Get().force_8_bit;
	bool force10bit = options_->Get().force_10_bit;
	
	// Decompression will require a buffer that's 8 pixels aligned.
	unsigned int buf_stride_pixels = item.info.width;
	unsigned int buf_stride_pixels_padded = (buf_stride_pixels + 7) & ~7;
	double bytesPerPixel = (double)bayer_format.bits / 8.0;
	int bitsPerPixel = bayer_format.bits;
	if(force8bit) {
		bytesPerPixel = 1;
	} else if (force10bit) {
		bytesPerPixel = 1.25;
	}
	std::vector<uint8_t> buf8bit(int(item.info.width * bytesPerPixel * item.info.height));
	std::vector<uint16_t> buf16Bit(buf_stride_pixels_padded * item.info.height);
	
	// Unpack/process the raw data
	if (bayer_format.compressed)
	{
		uncompress((uint8_t const*)item.mem, item.info, &buf16Bit[0]);
		buf_stride_pixels = buf_stride_pixels_padded;
	}
	else if (bayer_format.packed)
	{
		bitsPerPixel = bayer_format.bits;
		if(force8bit) {
			bitsPerPixel = 8;
		} else if(force10bit) {
			bitsPerPixel = 10;
		}
		switch (bayer_format.bits)
		{
		case 10:
			unpack_10bit((uint8_t const*)item.mem, item.info, &buf8bit[0], &buf16Bit[0]);
			break;
		case 12:
			if(force8bit) {
				unpack_12bit_to_8bit((uint8_t const*)item.mem, item.info, &buf8bit[0], &buf16Bit[0]);
			} else if (force10bit) {
				unpack_12bit_to_10bit((uint8_t const*)item.mem, item.info, &buf8bit[0], &buf16Bit[0]);
			} else {
				unpack_12bit((uint8_t const*)item.mem, item.info, &buf8bit[0], &buf16Bit[0]);
			}
			break;
		}
	}
	else {
		if( bitsPerPixel == 8 ) {
			copy_8bit((uint8_t const*)item.mem, item.info, &buf8bit[0], &buf16Bit[0]);
		} else {
			unpack_16bit((uint8_t const*)item.mem, item.info, &buf16Bit[0]);
		}
	}
	
	// Calculate black levels
	float black = 4096 * (1 << bayer_format.bits) / 65536.0;
	if(force8bit) {
		black = 16 + 12;
	} else if (force10bit) {
		black = 64 + 4;
	}
	float black_levels[] = { black, black, black, black };
	auto bl = item.control_list_metadata(libcamera::controls::SensorBlackLevels);
	if (bl)
	{
		for (int i = 0; i < 4; i++)
		{
			int j = bayer_format.order[i];
			j = j == 0 ? 0 : (j == 2 ? 3 : 1 + !!bayer_format.order[i ^ 1]);
			black_levels[j] = (*bl)[i] * (1 << bayer_format.bits) / 65536.0;
		}
	}
	else
		LOG_ERROR("WARNING: no black level found, using default");
	
	// Get exposure time
	auto exp = item.control_list_metadata.get(libcamera::controls::ExposureTime);
	float exp_time = 10000;
	if (exp)
		exp_time = *exp;
	else
		LOG_ERROR("WARNING: default to exposure time of " << exp_time << "us");
	exp_time /= 1e6;
	
	// Get ISO
	auto ag = item.control_list_metadata.get(libcamera::controls::AnalogueGain);
	uint16_t iso = 100;
	if (ag)
		iso = *ag * 100.0;
	else
		LOG_ERROR("WARNING: default to ISO value of " << iso);
	
	// White balance
	float NEUTRAL[] = { 1, 1, 1 };
	Matrix WB_GAINS(1, 1, 1);
	auto cg = item.control_list_metadata.get(libcamera::controls::ColourGains);
	if (cg)
	{
		NEUTRAL[0] = 1.0 / (*cg)[0];
		NEUTRAL[2] = 1.0 / (*cg)[1];
		WB_GAINS = Matrix((*cg)[0], 1, (*cg)[1]);
	}
	
	// CCM
	Matrix CCM(1.90255, -0.77478, -0.12777,
			   -0.31338, 1.88197, -0.56858,
			   -0.06001, -0.61785, 1.67786);
	auto ccm = item.control_list_metadata.get(libcamera::controls::ColourCorrectionMatrix);
	if (ccm)
	{
		CCM = Matrix((*ccm)[0], (*ccm)[1], (*ccm)[2], (*ccm)[3], (*ccm)[4], (*ccm)[5], (*ccm)[6], (*ccm)[7], (*ccm)[8]);
	}
	else
		LOG_ERROR("WARNING: no CCM metadata found");
	
	// Color matrix
	Matrix RGB2XYZ(0.4124564, 0.3575761, 0.1804375,
				   0.2126729, 0.7151522, 0.0721750,
				   0.0193339, 0.1191920, 0.9503041);
	Matrix CAM_XYZ = (RGB2XYZ * CCM * WB_GAINS).Inv();
	
	// Initialize memory buffer for TIFF
	TiffMemoryBuffer mem_buffer = { nullptr, 0, 0, 0 };
	mem_buffer.capacity = item.info.width * item.info.height * 3; // Initial estimate
	mem_buffer.data = (uint8_t *)malloc(mem_buffer.capacity);
	if (!mem_buffer.data)
		throw std::runtime_error("failed to allocate DNG memory buffer");
	mem_buffer.size = 0;
	mem_buffer.position = 0;
	
	TIFF *tif = nullptr;
	
	try
	{
		// Open TIFF with custom I/O for memory
		tif = TIFFClientOpen("memory", "w", &mem_buffer,
							 tiff_read, tiff_write, tiff_seek,
							 tiff_close, tiff_size, tiff_map, tiff_unmap);
		if (!tif)
			throw std::runtime_error("could not open TIFF for memory writing");
		
		short cfa_repeat_pattern_dim[] = { 2, 2 };
		if(options_->Get().monochrome) {
			cfa_repeat_pattern_dim[0] = 1;
			cfa_repeat_pattern_dim[1] = 1;
		}
		uint32_t white = (1 << bayer_format.bits) - 1;
		toff_t offset_subifd = 0, offset_exififd = 0;
		std::string unique_model = std::string(MAKE_STRING " shadowgraph-v3");
		
		unsigned int thumbnailSizeMultiplier = 3;
		// Thumbnail IFD
		TIFFSetField(tif, TIFFTAG_SUBFILETYPE, 1);
		TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, item.info.width >> thumbnailSizeMultiplier);
		TIFFSetField(tif, TIFFTAG_IMAGELENGTH, item.info.height >> thumbnailSizeMultiplier);
		TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 8);
		TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_NONE);
		TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
		TIFFSetField(tif, TIFFTAG_MAKE, MAKE_STRING);
		TIFFSetField(tif, TIFFTAG_MODEL, "shadowgraph-v3");
		TIFFSetField(tif, TIFFTAG_DNGVERSION, "\001\001\000\000");
		TIFFSetField(tif, TIFFTAG_DNGBACKWARDVERSION, "\001\000\000\000");
		TIFFSetField(tif, TIFFTAG_UNIQUECAMERAMODEL, unique_model.c_str());
		TIFFSetField(tif, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
		TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 3);
		TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
		TIFFSetField(tif, TIFFTAG_SOFTWARE, "shadowgraph-v3");
		TIFFSetField(tif, TIFFTAG_COLORMATRIX1, 9, CAM_XYZ.m);
		TIFFSetField(tif, TIFFTAG_ASSHOTNEUTRAL, 3, NEUTRAL);
		TIFFSetField(tif, TIFFTAG_CALIBRATIONILLUMINANT1, 21);
		TIFFSetField(tif, TIFFTAG_SUBIFD, 1, &offset_subifd);
		TIFFSetField(tif, TIFFTAG_EXIFIFD, offset_exififd);
		
		// Write thumbnail
		std::vector<uint8_t> thumb_buf((item.info.width >> thumbnailSizeMultiplier) * 3);
		for (unsigned int y = 0; y < (item.info.height >> thumbnailSizeMultiplier); y++)
		{
			for (unsigned int x = 0; x < (item.info.width >> thumbnailSizeMultiplier); x++)
			{
				unsigned int off = (y * buf_stride_pixels + x) << thumbnailSizeMultiplier;
				uint32_t grey = buf16Bit[off] + buf16Bit[off + 1] + buf16Bit[off + buf_stride_pixels] + buf16Bit[off + buf_stride_pixels + 1];
				grey = (grey << 14) >> bayer_format.bits;
				grey = sqrt((double)grey);
				thumb_buf[3 * x] = thumb_buf[3 * x + 1] = thumb_buf[3 * x + 2] = grey;
			}
			if (TIFFWriteScanline(tif, &thumb_buf[0], y, 0) != 1)
				throw std::runtime_error("error writing DNG thumbnail data");
		}
		
		TIFFWriteDirectory(tif);
		
		// ROI calculations
		unsigned int startX = (float)item.info.width * options_->Get().roi_x;
		unsigned int startY = (float)item.info.height * options_->Get().roi_y;
		unsigned int width = (float)item.info.width * options_->Get().roi_width;
		unsigned int height = (float)item.info.height * options_->Get().roi_height;
		
		if(bitsPerPixel == 10) {
			startX -= startX % 4;
		} else if (bitsPerPixel == 12) {
			startX -= startX % 2;
		}
		
		if(width == 0) {
			width = item.info.width - startX;
		}
		if(height == 0) {
			height = item.info.height;
		}
		
		unsigned int endX = startX + width;
		unsigned int endY = startY + height;
		
		if(endX > item.info.width) {
			endX = item.info.width;
			width = endX - startX;
		}
		if(endY > item.info.height) {
			endY = item.info.height;
			height = endY - startY;
		}
		
		// Main image IFD
		TIFFSetField(tif, TIFFTAG_SUBFILETYPE, 0);
		TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, width);
		TIFFSetField(tif, TIFFTAG_IMAGELENGTH, height);
		TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, bitsPerPixel);
		if(options_->Get().monochrome) {
			TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
		} else {
			TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_CFA);
		}
		TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
		TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
		TIFFSetField(tif, TIFFTAG_CFAREPEATPATTERNDIM, cfa_repeat_pattern_dim);
#if TIFFLIB_VERSION >= 20201219
		if(options_->Get().monochrome) {
			TIFFSetField(tif, TIFFTAG_CFAPATTERN, 4, TIFF_MONO);
		} else {
			TIFFSetField(tif, TIFFTAG_CFAPATTERN, 4, bayer_format.order);
		}
#else
		if(options_->Get().monochrome) {
			TIFFSetField(tif, TIFFTAG_CFAPATTERN, TIFF_MONO);
		} else {
			TIFFSetField(tif, TIFFTAG_CFAPATTERN, bayer_format.order);
		}
#endif
		TIFFSetField(tif, TIFFTAG_WHITELEVEL, 1, &white);
		const uint16_t black_level_repeat_dim[] = { 2, 2 };
		TIFFSetField(tif, TIFFTAG_BLACKLEVELREPEATDIM, &black_level_repeat_dim);
		TIFFSetField(tif, TIFFTAG_BLACKLEVEL, 4, &black_levels);
		
		// Write main image data
		unsigned int rowNum = 0;
		for (unsigned int y = startY; y < endY; y++)
		{
			unsigned int rowStartLocation = item.info.width * bytesPerPixel * y;
			unsigned int roiOffset = startX * bytesPerPixel;
			if (TIFFWriteScanline(tif, &buf8bit[rowStartLocation + roiOffset], rowNum, 0) != 1)
				throw std::runtime_error("error writing DNG image data");
			rowNum++;
		}
		
		TIFFCheckpointDirectory(tif);
		offset_subifd = TIFFCurrentDirOffset(tif);
		TIFFWriteDirectory(tif);
		
		// EXIF IFD
		TIFFCreateEXIFDirectory(tif);
		time_t t;
		time(&t);
		struct tm *time_info = localtime(&t);
		char time_str[32];
		strftime(time_str, 32, "%Y:%m:%d %H:%M:%S", time_info);
		TIFFSetField(tif, EXIFTAG_DATETIMEORIGINAL, time_str);
		TIFFSetField(tif, EXIFTAG_ISOSPEEDRATINGS, 1, &iso);
		TIFFSetField(tif, EXIFTAG_EXPOSURETIME, exp_time);
		
		auto lp = item.control_list_metadata.get(libcamera::controls::LensPosition);
		if (lp)
		{
			double dist = (*lp > 0.0) ? (1.0 / *lp) : std::numeric_limits<double>::infinity();
			TIFFSetField(tif, EXIFTAG_SUBJECTDISTANCE, dist);
		}
		
		TIFFCheckpointDirectory(tif);
		offset_exififd = TIFFCurrentDirOffset(tif);
		TIFFWriteDirectory(tif);
		
		// Update main IFD with offsets
		TIFFSetDirectory(tif, 0);
		TIFFSetField(tif, TIFFTAG_SUBIFD, 1, &offset_subifd);
		TIFFSetField(tif, TIFFTAG_EXIFIFD, offset_exififd);
		TIFFWriteDirectory(tif);
		
		TIFFUnlinkDirectory(tif, 2);
		TIFFClose(tif);
		tif = nullptr;
		
		// Transfer ownership of buffer
		encoded_buffer = mem_buffer.data;
		buffer_len = mem_buffer.size;
		mem_buffer.data = nullptr;
	}
	catch (std::exception const &e)
	{
		if (mem_buffer.data)
			free(mem_buffer.data);
		if (tif)
			TIFFClose(tif);
		throw;
	}
}

void DngEncoder::encodeThread(int num)
{
	std::chrono::duration<double> encode_time(0);
	uint32_t frames = 0;
	
	EncodeItem encode_item;
	while (true)
	{
		{
			std::unique_lock<std::mutex> lock(encode_mutex_);
			while (true)
			{
				using namespace std::chrono_literals;
				if (abortEncode_ && encode_queue_.empty())
				{
					if (frames)
						LOG(2, "Encode " << frames << " frames, average time " << encode_time.count() * 1000 / frames
										 << "ms");
					return;
				}
				if (!encode_queue_.empty())
				{
					encode_item = encode_queue_.front();
					encode_queue_.pop();
					break;
				}
				else
					encode_cond_var_.wait_for(lock, 200ms);
			}
		}
		
		// Encode the buffer
		uint8_t *encoded_buffer = nullptr;
		size_t buffer_len = 0;
		auto start_time = std::chrono::high_resolution_clock::now();
		try
		{
			encodeDNG(encode_item, encoded_buffer, buffer_len);
		}
		catch (std::exception const &e)
		{
			LOG_ERROR("DNG encoding error: " << e.what());
			// Continue to next frame
			continue;
		}
		encode_time += (std::chrono::high_resolution_clock::now() - start_time);
		frames++;
		
		// Push encoded buffer to output queue
		OutputItem output_item = { encoded_buffer, buffer_len, encode_item.timestamp_us, encode_item.index };
		std::lock_guard<std::mutex> lock(output_mutex_);
		output_queue_[num].push(output_item);
		output_cond_var_.notify_one();
	}
}

void DngEncoder::outputThread()
{
	OutputItem item;
	uint64_t index = 0;
	while (true)
	{
		{
			std::unique_lock<std::mutex> lock(output_mutex_);
			while (true)
			{
				using namespace std::chrono_literals;
				bool abort = abortOutput_ ? true : false;
				for (auto &q : output_queue_)
				{
					if (abort && !q.empty())
						abort = false;
					
					if (!q.empty() && q.front().index == index)
					{
						item = q.front();
						q.pop();
						goto got_item;
					}
				}
				if (abort)
					return;
				
				output_cond_var_.wait_for(lock, 200ms);
			}
		}
	got_item:
		input_done_callback_(nullptr);
		
		output_ready_callback_(item.mem, item.bytes_used, item.timestamp_us, true);
		free(item.mem);
		index++;
	}
}
