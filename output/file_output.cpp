/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * file_output.cpp - Write output to file.
 */
#include <filesystem>
#include <string>

#include "file_output.hpp"
#include "file_name_manager.hpp"
#include "image/image.hpp"
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include "core/still_options.hpp"
#include "core/stream_info.hpp"
#include "core/options.hpp"

namespace fs = std::filesystem;

FileOutput::FileOutput(VideoOptions const *options)
	: Output(options), fp_(nullptr), count_(0), file_start_time_ms_(0), fileNameManager_((Options*)options)
{
	// fileNameManager_ = FileNameManager((Options*)options);
}

FileOutput::~FileOutput()
{
	closeFile();
}

// void FileOutput::outputBuffer(void *mem, size_t size, int64_t timestamp_us, uint32_t flags)
// {
// 	if(current_directory_size_ > options_->max_directory_size) {
// 		makeNewCurrentDir();
// 	}
// 	// We need to open a new file if we're in "segment" mode and our segment is full
// 	// (though we have to wait for the next I frame), or if we're in "split" mode
// 	// and recording is being restarted (this is necessarily an I-frame already).
// 	if (fp_ == nullptr ||
// 		(options_->segment && (flags & FLAG_KEYFRAME) &&
// 		 timestamp_us / 1000 - file_start_time_ms_ > options_->segment) ||
// 		(options_->split && (flags & FLAG_RESTART)))
// 	{
// 		closeFile();
// 		openFile(timestamp_us);
// 	}

// 	LOG(2, "FileOutput: output buffer " << mem << " size " << size);
// 	if (fp_ && size)
// 	{
// 		if (fwrite(mem, size, 1, fp_) != 1)
// 			throw std::runtime_error("failed to write output bytes");
// 		if (options_->flush)
// 			fflush(fp_);
// 	}
// 	current_directory_size_++;
// }

void FileOutput::outputBuffer(void *mem, size_t size, int64_t timestamp_us, uint32_t flags)
{
	libcamera::ControlList mockControlList;

	StreamInfo mockInfo;
	mockInfo.width = 4056;
	mockInfo.height = 3040;
	mockInfo.stride = 6112;
	mockInfo.pixel_format = libcamera::formats::SBGGR12_CSI2P;

	std::string fileNameString = fileNameManager_.getNextFileName();

	dng_save(mem, mockInfo, mockControlList, fileNameString, "mock-camera-model", NULL);
}

void FileOutput::openFile(int64_t timestamp_us)
{
	if (options_->output == "-")
		fp_ = stdout;
	else if (!options_->output.empty())
	{
		// Generate the next output file name.
		// We should expect a filename to be build by the parentDir + current_directory + output file name
		fs::path pathToCurrentDir = fs::path(options_->parent_directory) / current_directory_;
		fs::path pathToFile = pathToCurrentDir / options_->output;
		char filename[256];
		int n = snprintf(filename, sizeof(filename), pathToFile.string().c_str(), count_);
		count_++;
		if (options_->wrap)
			count_ = count_ % options_->wrap;
		if (n < 0)
			throw std::runtime_error("failed to generate filename");

		fp_ = fopen(filename, "w");
		if (!fp_)
			throw std::runtime_error("failed to open output file " + std::string(filename));
		LOG(2, "FileOutput: opened output file " << filename);

		file_start_time_ms_ = timestamp_us / 1000;
	}
}

void FileOutput::closeFile()
{
	if (fp_)
	{
		if (options_->flush)
			fflush(fp_);
		if (fp_ != stdout)
			fclose(fp_);
		fp_ = nullptr;
	}
}
