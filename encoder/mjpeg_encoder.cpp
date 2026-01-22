/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * mjpeg_encoder.cpp - mjpeg video encoder.
 */

#include <chrono>
#include <iostream>
#include <cstring>
#include <ctime>

#include <jpeglib.h>
#include <libexif/exif-data.h>
#include <libcamera/control_ids.h>

#include "mjpeg_encoder.hpp"
#include "core/logging.hpp"
#include "core/metadata.hpp"

#ifndef MAKE_STRING
#define MAKE_STRING "Wassoc"
#endif

static const ExifByteOrder exif_byte_order = EXIF_BYTE_ORDER_INTEL;
static const unsigned int exif_image_offset = 20; // offset of image in JPEG buffer
static const unsigned char exif_header[] = { 0xff, 0xd8, 0xff, 0xe1 };

#if JPEG_LIB_VERSION_MAJOR > 9 || (JPEG_LIB_VERSION_MAJOR == 9 && JPEG_LIB_VERSION_MINOR >= 4)
typedef size_t jpeg_mem_len_t;
#else
typedef unsigned long jpeg_mem_len_t;
#endif

MjpegEncoder::MjpegEncoder(VideoOptions const *options)
	: Encoder(options), abortEncode_(false), abortOutput_(false), index_(0)
{
	output_thread_ = std::thread(&MjpegEncoder::outputThread, this);
	for (int i = 0; i < NUM_ENC_THREADS; i++)
		encode_thread_[i] = std::thread(std::bind(&MjpegEncoder::encodeThread, this, i));
	LOG(2, "Opened MjpegEncoder");
}

MjpegEncoder::~MjpegEncoder()
{
	abortEncode_ = true;
	for (int i = 0; i < NUM_ENC_THREADS; i++)
		encode_thread_[i].join();
	abortOutput_ = true;
	output_thread_.join();
	LOG(2, "MjpegEncoder closed");
}

void MjpegEncoder::EncodeBuffer(int fd, size_t size, void *mem, StreamInfo const &info, int64_t timestamp_us, Metadata const &metadata)
{
	std::lock_guard<std::mutex> lock(encode_mutex_);
	EncodeItem item = { mem, info, timestamp_us, index_++, metadata };
	encode_queue_.push(item);
	encode_cond_var_.notify_all();
}

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

// Create EXIF data from metadata
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

		// Add basic EXIF tags to IFD0 (main image directory) for better Windows compatibility
		ExifEntry *entry = exif_create_tag(exif, EXIF_IFD_0, EXIF_TAG_MAKE);
		exif_set_string(entry, MAKE_STRING);
		// Add MODEL tag - Windows Explorer often looks for this
		entry = exif_create_tag(exif, EXIF_IFD_0, EXIF_TAG_MODEL);
		exif_set_string(entry, "Shadowgraph-v3"); // Generic model name
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
		std::string camera_serial_number = "Unknown";
		auto camera_serial_number_defined = metadata.Get(std::string("exif_data.camera_serial_number"), camera_serial_number);
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

void MjpegEncoder::encodeJPEG(struct jpeg_compress_struct &cinfo, EncodeItem &item, uint8_t *&encoded_buffer,
							  size_t &buffer_len)
{
	// Copied from YUV420_to_JPEG_fast in jpeg.cpp.
	cinfo.image_width = item.info.width;
	cinfo.image_height = item.info.height;
	cinfo.input_components = 3;
	cinfo.in_color_space = JCS_YCbCr;
	cinfo.restart_interval = 0;

	jpeg_set_defaults(&cinfo);
	cinfo.raw_data_in = TRUE;
	jpeg_set_quality(&cinfo, options_->Get().quality, TRUE);
	encoded_buffer = nullptr;
	buffer_len = 0;
	jpeg_mem_len_t jpeg_mem_len;
	jpeg_mem_dest(&cinfo, &encoded_buffer, &jpeg_mem_len);
	jpeg_start_compress(&cinfo, TRUE);
	
	// Add EXIF metadata if available
	uint8_t *exif_buffer = nullptr;
	unsigned int exif_len = 0;
	std::string temp_lamp_color;
	if (item.metadata.Get(std::string("exif_data.lamp_color"), temp_lamp_color) == 0)
	{
		try
		{
			create_exif_data(item.metadata, exif_buffer, exif_len);
			if (exif_buffer && exif_len > 0)
			{
				// Check if libexif already includes "Exif\0\0" prefix
				// libexif's exif_data_save_data should return data starting with TIFF header (0x4949 or 0x4D4D)
				// JPEG APP1 requires "Exif\0\0" prefix before TIFF data
				bool needs_prefix = (exif_len < 6 || memcmp(exif_buffer, "Exif", 4) != 0);
				if (needs_prefix)
				{
					// JPEG APP1 EXIF format requires "Exif\0\0" prefix
					// libexif's exif_data_save_data returns TIFF data without this prefix
					// So we need to prepend it
					uint8_t exif_header[6] = { 'E', 'x', 'i', 'f', 0, 0 };
					uint8_t *exif_with_header = (uint8_t *)malloc(6 + exif_len);
					if (exif_with_header)
					{
						memcpy(exif_with_header, exif_header, 6);
						memcpy(exif_with_header + 6, exif_buffer, exif_len);
						// Write EXIF as APP1 segment
						jpeg_write_marker(&cinfo, JPEG_APP0 + 1, exif_with_header, 6 + exif_len);
						free(exif_with_header);
					}
				}
				else
				{
					// Already has prefix, write directly
					jpeg_write_marker(&cinfo, JPEG_APP0 + 1, exif_buffer, exif_len);
				}
				LOG(2, "Wrote EXIF marker, size: " << (needs_prefix ? exif_len + 6 : exif_len));
			}
		}
		catch (std::exception const &e)
		{
			LOG_ERROR("Failed to create EXIF data: " << e.what());
			// Continue without EXIF
		}
	}

	int stride2 = item.info.stride / 2;
	uint8_t *Y = (uint8_t *)item.mem;
	uint8_t *U = (uint8_t *)Y + item.info.stride * item.info.height;
	uint8_t *V = (uint8_t *)U + stride2 * (item.info.height / 2);
	uint8_t *Y_max = U - item.info.stride;
	uint8_t *U_max = V - stride2;
	uint8_t *V_max = U_max + stride2 * (item.info.height / 2);

	JSAMPROW y_rows[16];
	JSAMPROW u_rows[8];
	JSAMPROW v_rows[8];

	for (uint8_t *Y_row = Y, *U_row = U, *V_row = V; cinfo.next_scanline < item.info.height;)
	{
		for (int i = 0; i < 16; i++, Y_row += item.info.stride)
			y_rows[i] = std::min(Y_row, Y_max);
		for (int i = 0; i < 8; i++, U_row += stride2, V_row += stride2)
			u_rows[i] = std::min(U_row, U_max), v_rows[i] = std::min(V_row, V_max);

		JSAMPARRAY rows[] = { y_rows, u_rows, v_rows };
		jpeg_write_raw_data(&cinfo, rows, 16);
	}

	jpeg_finish_compress(&cinfo);
	buffer_len = jpeg_mem_len;
	
	// Cleanup EXIF buffer
	if (exif_buffer)
		free(exif_buffer);
}

void MjpegEncoder::encodeThread(int num)
{
	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;
	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&cinfo);
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
					jpeg_destroy_compress(&cinfo);
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

		// Encode the buffer.
		uint8_t *encoded_buffer = nullptr;
		size_t buffer_len = 0;
		auto start_time = std::chrono::high_resolution_clock::now();
		encodeJPEG(cinfo, encode_item, encoded_buffer, buffer_len);
		encode_time += (std::chrono::high_resolution_clock::now() - start_time);
		frames++;
		// Don't return buffers until the output thread as that's where they're
		// in order again.

		// We push this encoded buffer to another thread so that our
		// application can take its time with the data without blocking the
		// encode process.
		OutputItem output_item = { encoded_buffer, buffer_len, encode_item.timestamp_us, encode_item.index };
		std::lock_guard<std::mutex> lock(output_mutex_);
		output_queue_[num].push(output_item);
		output_cond_var_.notify_one();
	}
}

void MjpegEncoder::outputThread()
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
