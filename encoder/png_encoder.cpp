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
#include <libexif/exif-data.h>
#include <libcamera/control_ids.h>
#include <ctime>

#include "png_encoder.hpp"
#include "core/logging.hpp"
#include "core/metadata.hpp"

#ifndef MAKE_STRING
#define MAKE_STRING "Wassoc"
#endif

static const ExifByteOrder exif_byte_order = EXIF_BYTE_ORDER_INTEL;

// Helper function to create EXIF entry
static ExifEntry *exif_create_tag(ExifData *exif, ExifIfd ifd, ExifTag tag)
{
	ExifEntry *entry = exif_content_get_entry(exif->ifd[ifd], tag);
	if (entry)
		return entry;
	entry = exif_entry_new();
	if (!entry)
		throw std::runtime_error("failed to allocate EXIF entry");
	entry->tag = tag;
	exif_content_add_entry(exif->ifd[ifd], entry);
	exif_entry_initialize(entry, entry->tag);
	// Ensure data is allocated if entry_initialize didn't do it
	if (entry->size > 0 && !entry->data)
	{
		entry->data = (unsigned char *)malloc(entry->size);
		if (!entry->data)
			throw std::runtime_error("failed to allocate EXIF entry data");
		memset(entry->data, 0, entry->size);
	}
	exif_entry_unref(entry);
	return entry;
}

// Helper function to set EXIF string
static void exif_set_string(ExifEntry *entry, char const *s)
{
	if (entry->data)
		free(entry->data);
	entry->size = entry->components = strlen(s);
	entry->data = (unsigned char *)strdup(s);
	if (!entry->data)
		throw std::runtime_error("failed to copy exif string");
	entry->format = EXIF_FORMAT_ASCII;
}

// Create EXIF data from metadata (mimics mjpeg_encoder.cpp)
static void create_exif_data(Metadata const &metadata, uint8_t *&exif_buffer, unsigned int &exif_len)
{
	exif_buffer = nullptr;
	ExifData *exif = nullptr;

	try
	{
		exif = exif_data_new();
		if (!exif)
			throw std::runtime_error("failed to allocate EXIF data");
		exif_data_set_byte_order(exif, exif_byte_order);

		std::string camera_serial_number = "Unknown";
		auto camera_serial_number_defined = metadata.Get(std::string("exif_data.camera_serial_number"), camera_serial_number);

		// Add basic EXIF tags to IFD0 (main image directory) for better Windows compatibility
		ExifEntry *entry = exif_create_tag(exif, EXIF_IFD_0, EXIF_TAG_MAKE);
		exif_set_string(entry, MAKE_STRING);
		// Add MODEL tag - Windows Explorer often looks for this
		entry = exif_create_tag(exif, EXIF_IFD_0, EXIF_TAG_MODEL);
		exif_set_string(entry, std::string("Shadowgraph-v3 (SN: " + camera_serial_number + ")").c_str());
		entry = exif_create_tag(exif, EXIF_IFD_0, EXIF_TAG_SOFTWARE);
		exif_set_string(entry, "Shadowgraph-v3");
		
		// Add date/time to IFD0 for Windows Explorer compatibility
		std::time_t raw_time;
		std::time(&raw_time);
		std::tm *time_info = std::localtime(&raw_time);
		char time_string[32];
		std::strftime(time_string, sizeof(time_string), "%Y:%m:%d %H:%M:%S", time_info);
		entry = exif_create_tag(exif, EXIF_IFD_0, EXIF_TAG_DATE_TIME);
		exif_set_string(entry, time_string);
		entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_DATE_TIME_ORIGINAL);
		exif_set_string(entry, time_string);
		entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_DATE_TIME_DIGITIZED);
		exif_set_string(entry, time_string);

		// Add exposure time (shutter speed) - Windows Explorer expects this in EXIF sub-IFD
		float exposure_time;
		auto exposure_time_defined = metadata.Get(std::string("exif_data.shutter_speed"), exposure_time);
		if (exposure_time_defined == 0)
		{
			entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_EXPOSURE_TIME);
			ExifRational exposure = { (ExifLong)exposure_time, 1000000 };
			exif_set_rational(entry->data, exif_byte_order, exposure);
		}

		// Add ISO (from gains) - Windows Explorer expects this in EXIF sub-IFD
		float ag = 1.0;
		auto agDefined = metadata.Get(std::string("exif_data.analogue_gain"), ag);
		if (agDefined == 0)
		{
			entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_ISO_SPEED_RATINGS);
		}

		float dg = 1.0;
		auto dgDefined = metadata.Get(std::string("exif_data.digital_gain"), dg);
		if (dgDefined == 0)
		{
			float gain = ag * (dgDefined == 0 ? dg : 1.0);
			exif_set_short(entry->data, exif_byte_order, (ExifShort)(100 * gain));
		}

		// Add fixed f-stop (aperture) value of f/16 to EXIF metadata
		entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_FNUMBER);
		// EXIF f-stop is a rational value: numerator=focal/aperture, denominator=1 (for whole numbers)
		// For f/16, value is 16/1
		ExifRational fnumber = { 16, 1 }; // f/16
		exif_set_rational(entry->data, exif_byte_order, fnumber);

		// Add lamp color to EXIF metadata as user comment
		std::string lamp_color = "Unknown";
		auto lampDefined = metadata.Get(std::string("exif_data.lamp_color"), lamp_color);
		if (lampDefined == 0) {
			entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_USER_COMMENT);
			exif_set_string(entry, std::string("Lamp color: " + lamp_color).c_str());
		}

		// Set focal length to 12mm in EXIF
		entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_FOCAL_LENGTH);
		// EXIF focal length is a rational value, so 12/1 = 12mm
		ExifRational focal_length = { 12, 1 };
		exif_set_rational(entry->data, exif_byte_order, focal_length);

		// Add camera serial number to EXIF metadata
		// Try IFD0 first for better Windows Explorer compatibility
		if (camera_serial_number_defined == 0 && !camera_serial_number.empty()) {
			// Try in IFD0 for Windows Explorer compatibility
			entry = exif_create_tag(exif, EXIF_IFD_0, EXIF_TAG_BODY_SERIAL_NUMBER);
			exif_set_string(entry, camera_serial_number.c_str());
			// Also set in EXIF sub-IFD for standard compliance
			entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_BODY_SERIAL_NUMBER);
			exif_set_string(entry, camera_serial_number.c_str());
		}

		// Create the EXIF data buffer
		// libexif should automatically set up the EXIF sub-IFD pointer when we add tags to EXIF_IFD_EXIF
		exif_data_save_data(exif, &exif_buffer, &exif_len);
		if (!exif_buffer || exif_len == 0)
			throw std::runtime_error("failed to save EXIF data");
		LOG(2, "Created EXIF data, length: " << exif_len);
		exif_data_unref(exif);
		exif = nullptr;
	}
	catch (std::exception const &e)
	{
		if (exif)
			exif_data_unref(exif);
		if (exif_buffer)
			free(exif_buffer);
		throw;
	}
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

void PngEncoder::EncodeBuffer(int fd, size_t size, void *mem, StreamInfo const &info, int64_t timestamp_us, Metadata const &metadata)
{
	std::lock_guard<std::mutex> lock(encode_mutex_);
	EncodeItem item = { mem, info, timestamp_us, index_++, metadata };
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

		// Add EXIF metadata as PNG eXIf chunk (mimics mjpeg_encoder.cpp)
		std::string temp_lamp_color;
		if (item.metadata.Get(std::string("exif_data.lamp_color"), temp_lamp_color) == 0)
		{
			try
			{
				uint8_t *exif_buffer = nullptr;
				unsigned int exif_len = 0;
				create_exif_data(item.metadata, exif_buffer, exif_len);
				if (exif_buffer && exif_len > 0)
				{
					// PNG eXIf chunk requires "Exif\0\0" prefix before TIFF data
					// libexif's exif_data_save_data returns TIFF data without this prefix
					bool needs_prefix = (exif_len < 6 || memcmp(exif_buffer, "Exif", 4) != 0);
					if (needs_prefix)
					{
						// PNG eXIf format requires "Exif\0\0" prefix
						uint8_t exif_header[6] = { 'E', 'x', 'i', 'f', 0, 0 };
						exif_data_storage.resize(6 + exif_len);
						memcpy(exif_data_storage.data(), exif_header, 6);
						memcpy(exif_data_storage.data() + 6, exif_buffer, exif_len);
						free(exif_buffer);
					}
					else
					{
						// Already has prefix, copy directly
						exif_data_storage.resize(exif_len);
						memcpy(exif_data_storage.data(), exif_buffer, exif_len);
						free(exif_buffer);
					}
					
					// Create unknown chunk for EXIF
					png_unknown_chunk exif_chunk;
					memcpy(exif_chunk.name, "eXIf", 5); // 5 bytes: "eXIf" + null terminator
					exif_chunk.data = exif_data_storage.data();
					exif_chunk.size = exif_data_storage.size();
					exif_chunk.location = PNG_HAVE_IHDR;
					
					// Add the EXIF chunk
					png_set_unknown_chunks(png_ptr, info_ptr, &exif_chunk, 1);
					png_set_unknown_chunk_location(png_ptr, info_ptr, 0, PNG_HAVE_IHDR);
					LOG(2, "Wrote EXIF chunk, size: " << exif_data_storage.size());
				}
			}
			catch (std::exception const &e)
			{
				LOG_ERROR("Failed to create EXIF data: " << e.what());
				// Continue without EXIF
			}
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
