/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * png_three_photo_encoder.hpp - PNG encoder that combines three
 * monochrome frames (R, G, B) into a single RGB888 PNG.
 */

#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <libcamera/controls.h>

#include "encoder.hpp"
#include "core/metadata.hpp"

// Encodes three consecutive 8-bit monochrome frames into a single
// RGB888 PNG. The first frame contributes the red channel, the second
// the green channel, and the third the blue channel.
class PngThreePhotoEncoder : public Encoder
{
public:
	PngThreePhotoEncoder(VideoOptions const *options);
	~PngThreePhotoEncoder();

	// Encode a single monochrome buffer. Three calls to EncodeBuffer
	// will result in one RGB PNG being produced.
	void EncodeBuffer(int fd, size_t size, void *mem, StreamInfo const &info,
					  int64_t timestamp_us,
					  Metadata const &post_process_metadata = Metadata(),
					  libcamera::ControlList const &control_list_metadata = libcamera::ControlList()) override;

private:
	// Single input frame (monochrome) waiting to be grouped into RGB triplets.
	struct MonoFrameItem
	{
		void *mem;
		StreamInfo info;
		int64_t timestamp_us;
		uint64_t index;
		Metadata metadata;
	};

	// Item describing a completed RGB PNG in memory.
	struct OutputItem
	{
		void *mem;
		size_t bytes_used;
		int64_t timestamp_us;
		uint64_t index;
		// Number of input frames consumed to produce this output.
		unsigned int frames_used;
	};

	void encodeThread(int num);
	void outputThread();

	// Perform PNG encoding for a triplet of monochrome frames.
	void encodePNG(MonoFrameItem const &red,
				   MonoFrameItem const &green,
				   MonoFrameItem const &blue,
				   uint8_t *&encoded_buffer,
				   size_t &buffer_len);

	bool abortEncode_;
	bool abortOutput_;
	uint64_t index_;       // input frame index (per EncodeBuffer call)
	uint64_t output_index_; // sequential index for each RGB PNG output (0, 1, 2, ...)

	// We only support a single encoding thread because frames must be
	// grouped sequentially in triplets.
	static const int NUM_ENC_THREADS = 1;

	std::queue<MonoFrameItem> encode_queue_;
	std::mutex encode_mutex_;
	std::condition_variable encode_cond_var_;
	std::thread encode_thread_[NUM_ENC_THREADS];

	std::queue<OutputItem> output_queue_[NUM_ENC_THREADS];
	std::mutex output_mutex_;
	std::condition_variable output_cond_var_;
	std::thread output_thread_;

	VideoOptions const *options_;
};

