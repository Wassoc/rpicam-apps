/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * png_encoder.cpp - PNG video encoder.
 */

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <vector>
#include <algorithm>

#include <png.h>
#include <libcamera/control_ids.h>

#include "png_encoder.hpp"
#include "core/logging.hpp"

// Helper function to write little-endian values
// static void write_le16(uint8_t *buf, uint16_t value)
// {
// 	buf[0] = value & 0xFF;
// 	buf[1] = (value >> 8) & 0xFF;
// }

static void write_le32(uint8_t *buf, uint32_t value)
{
	buf[0] = value & 0xFF;
	buf[1] = (value >> 8) & 0xFF;
	buf[2] = (value >> 16) & 0xFF;
	buf[3] = (value >> 24) & 0xFF;
}

// Build a minimal EXIF block with exposure time
// EXIF structure: "Exif\0\0" + TIFF header + IFD with exposure time tag
static std::vector<uint8_t> BuildExifBlock(double exposure_seconds)
{
	std::vector<uint8_t> exif_data;
	
	// EXIF header: "Exif\0\0"
	exif_data.insert(exif_data.end(), { 'E', 'x', 'i', 'f', 0, 0 });
	
	// TIFF header (8 bytes)
	// Byte order: 0x4949 = Intel (little-endian)
	exif_data.insert(exif_data.end(), { 0x49, 0x49 });
	// TIFF version: 42
	exif_data.insert(exif_data.end(), { 0x2A, 0x00 });
	// Offset to first IFD: 8 (points to IFD right after header)
	exif_data.insert(exif_data.end(), { 0x08, 0x00, 0x00, 0x00 });
	
	// IFD (Image File Directory)
	// Number of directory entries: 1 (just exposure time)
	exif_data.insert(exif_data.end(), { 0x01, 0x00 });
	
	// EXIF_TAG_EXPOSURE_TIME = 0x829A
	// Tag ID (2 bytes)
	exif_data.insert(exif_data.end(), { 0x9A, 0x82 });
	// Type: RATIONAL (5) = 2 bytes
	exif_data.insert(exif_data.end(), { 0x05, 0x00 });
	// Count: 1 = 4 bytes
	exif_data.insert(exif_data.end(), { 0x01, 0x00, 0x00, 0x00 });
	
	// Value/Offset: For RATIONAL, this is an offset to the actual value
	// We'll put the value right after the IFD, so offset is current size + 4 (for next IFD offset)
	uint32_t value_offset = exif_data.size() + 4;
	uint8_t offset_bytes[4];
	write_le32(offset_bytes, value_offset);
	exif_data.insert(exif_data.end(), offset_bytes, offset_bytes + 4);
	
	// Next IFD offset: 0 (no more IFDs)
	exif_data.insert(exif_data.end(), { 0x00, 0x00, 0x00, 0x00 });
	
	// Exposure time value (RATIONAL: numerator, denominator)
	// Convert seconds to rational (e.g., 1/50 = 1, 50)
	uint32_t numerator, denominator;
	if (exposure_seconds < 1.0)
	{
		// For fractions, use 1/denominator format
		denominator = (uint32_t)(1.0 / exposure_seconds + 0.5);
		numerator = 1;
	}
	else
	{
		// For >= 1 second, use seconds/1 format
		numerator = (uint32_t)(exposure_seconds * 1000000 + 0.5);
		denominator = 1000000;
	}
	
	uint8_t num_bytes[4], den_bytes[4];
	write_le32(num_bytes, numerator);
	write_le32(den_bytes, denominator);
	exif_data.insert(exif_data.end(), num_bytes, num_bytes + 4);
	exif_data.insert(exif_data.end(), den_bytes, den_bytes + 4);
	
	return exif_data;
}

// Structure to hold memory buffer for PNG encoding
struct PngMemoryBuffer
{
	uint8_t *data;
	size_t size;
	size_t capacity;
};

// Custom write function for libpng to write to memory
static void png_write_to_memory(png_structp png_ptr, png_bytep data, png_size_t length)
{
	PngMemoryBuffer *buffer = (PngMemoryBuffer *)png_get_io_ptr(png_ptr);
	
	// Grow buffer if needed
	if (buffer->size + length > buffer->capacity)
	{
		size_t new_capacity = buffer->capacity * 2;
		if (new_capacity < buffer->size + length)
			new_capacity = buffer->size + length + 1024; // Add some extra space
		
		uint8_t *new_data = (uint8_t *)realloc(buffer->data, new_capacity);
		if (!new_data)
		{
			png_error(png_ptr, "failed to allocate memory for PNG buffer");
			return;
		}
		buffer->data = new_data;
		buffer->capacity = new_capacity;
	}
	
	memcpy(buffer->data + buffer->size, data, length);
	buffer->size += length;
}

// Custom flush function (no-op for memory)
static void png_flush_memory(png_structp png_ptr)
{
	(void)png_ptr;
}

PngEncoder::PngEncoder(VideoOptions const *options)
	: Encoder(options), abortEncode_(false), abortOutput_(false), index_(0)
{
	options_ = options;
	output_thread_ = std::thread(&PngEncoder::outputThread, this);
	for (int i = 0; i < NUM_ENC_THREADS; i++)
		encode_thread_[i] = std::thread(std::bind(&PngEncoder::encodeThread, this, i));
	LOG(2, "Opened PngEncoder");
}

PngEncoder::~PngEncoder()
{
	abortEncode_ = true;
	for (int i = 0; i < NUM_ENC_THREADS; i++)
		encode_thread_[i].join();
	abortOutput_ = true;
	output_thread_.join();
	LOG(2, "PngEncoder closed");
}

void PngEncoder::EncodeBuffer(int fd, size_t size, void *mem, StreamInfo const &info, int64_t timestamp_us)
{
	std::lock_guard<std::mutex> lock(encode_mutex_);
	EncodeItem item = { mem, info, timestamp_us, index_++ };
	encode_queue_.push(item);
	encode_cond_var_.notify_all();
}

void PngEncoder::encodePNG(EncodeItem &item, uint8_t *&encoded_buffer, size_t &buffer_len)
{
	png_structp png_ptr = NULL;
	png_infop info_ptr = NULL;
	PngMemoryBuffer mem_buffer = { nullptr, 0, 0 };
	std::vector<uint8_t> exif_data_storage; // Store EXIF data to keep it alive

	try
	{
		// Initialize memory buffer
		mem_buffer.capacity = item.info.width * item.info.height + 1024; // Initial estimate
		mem_buffer.data = (uint8_t *)malloc(mem_buffer.capacity);
		if (!mem_buffer.data)
			throw std::runtime_error("failed to allocate PNG memory buffer");
		mem_buffer.size = 0;

		// Create PNG structures
		png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
		if (png_ptr == NULL)
			throw std::runtime_error("failed to create png write struct");

		info_ptr = png_create_info_struct(png_ptr);
		if (info_ptr == NULL)
			throw std::runtime_error("failed to create png info struct");

		if (setjmp(png_jmpbuf(png_ptr)))
			throw std::runtime_error("failed to set png error handling");

		// Set image attributes
		png_set_IHDR(png_ptr, info_ptr, item.info.width, item.info.height, 8, PNG_COLOR_TYPE_GRAY,
					 PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_BASE);
		// These settings get us most of the compression, but are much faster.
		png_set_filter(png_ptr, PNG_FILTER_TYPE_BASE, PNG_FILTER_NONE);
		// Passing 0 to not compress the image
		png_set_compression_level(png_ptr, options_->Get().png_compression_level);

		// Add EXIF metadata as PNG eXIf chunk
		auto exposure_time = options_->Get().shutter;
		if (exposure_time)
		{
			double exposure_seconds = exposure_time.get<std::chrono::microseconds>() / 1000000.0;
			
			// Build EXIF data block and store it to keep it alive
			exif_data_storage = BuildExifBlock(exposure_seconds);
			
			// Create unknown chunk for EXIF
			png_unknown_chunk exif_chunk;
			memcpy(exif_chunk.name, "eXIf", 5); // 5 bytes: "eXIf" + null terminator
			exif_chunk.data = exif_data_storage.data();
			exif_chunk.size = exif_data_storage.size();
			exif_chunk.location = PNG_HAVE_IHDR;
			
			// Add the EXIF chunk
			png_set_unknown_chunks(png_ptr, info_ptr, &exif_chunk, 1);
			png_set_unknown_chunk_location(png_ptr, info_ptr, 0, PNG_HAVE_IHDR);
		}
			// Add shutter speed (exposure time)
			// auto exposure_time = item.metadata.get(libcamera::controls::ExposureTime);
			// if (exposure_time)
			// {
			// 	std::ostringstream oss;
			// 	oss << std::fixed << std::setprecision(6) << (*exposure_time / 1000000.0) << " s";
			// 	item.metadata_strings.push_back(oss.str());
			// 	png_text text;
			// 	text.compression = PNG_TEXT_COMPRESSION_NONE;
			// 	text.key = (png_charp)"ExposureTime";
			// 	text.text = (png_charp)item.metadata_strings.back().c_str();
			// 	text.text_length = item.metadata_strings.back().length();
			// 	text.itxt_length = 0;
			// 	text.lang = nullptr;
			// 	text.lang_key = nullptr;
			// 	text_chunks.push_back(text);
			// }

			// // Add analogue gain
			// auto ag = item.metadata.get(libcamera::controls::AnalogueGain);
			// if (ag)
			// {
			// 	std::ostringstream oss;
			// 	oss << std::fixed << std::setprecision(2) << *ag;
			// 	item.metadata_strings.push_back(oss.str());
			// 	png_text text;
			// 	text.compression = PNG_TEXT_COMPRESSION_NONE;
			// 	text.key = (png_charp)"AnalogueGain";
			// 	text.text = (png_charp)item.metadata_strings.back().c_str();
			// 	text.text_length = item.metadata_strings.back().length();
			// 	text.itxt_length = 0;
			// 	text.lang = nullptr;
			// 	text.lang_key = nullptr;
			// 	text_chunks.push_back(text);
			// }

		

		// Set up the image data
		png_byte **row_ptrs = (png_byte **)png_malloc(png_ptr, item.info.height * sizeof(png_byte *));
		png_byte *row = (uint8_t *)item.mem;
		for (unsigned int i = 0; i < item.info.height; i++, row += item.info.stride)
			row_ptrs[i] = row;

		// Use custom write function to write to memory
		png_set_write_fn(png_ptr, &mem_buffer, png_write_to_memory, png_flush_memory);
		png_set_rows(png_ptr, info_ptr, row_ptrs);
		png_write_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, NULL);

		// Transfer ownership of the buffer
		encoded_buffer = mem_buffer.data;
		buffer_len = mem_buffer.size;
		mem_buffer.data = nullptr; // Prevent free in cleanup

		// Cleanup
		png_free(png_ptr, row_ptrs);
		png_destroy_write_struct(&png_ptr, &info_ptr);
	}
	catch (std::exception const &e)
	{
		if (mem_buffer.data)
			free(mem_buffer.data);
		if (png_ptr)
			png_destroy_write_struct(&png_ptr, &info_ptr);
		throw;
	}
}

void PngEncoder::encodeThread(int num)
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
			encodePNG(encode_item, encoded_buffer, buffer_len);
		}
		catch (std::exception const &e)
		{
			LOG_ERROR("PNG encoding error: " << e.what());
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

void PngEncoder::outputThread()
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
				// We look for the thread that's completed the frame we want next.
				// If we don't find it, we wait.
				//
				// Must also check for an abort signal, and if set, all queues must
				// be empty. This is done first to ensure all frame callbacks have
				// had a chance to run.
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
