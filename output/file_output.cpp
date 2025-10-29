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
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;
using json = nlohmann::json;

FileOutput::FileOutput(VideoOptions const *options)
	: Output(options), fp_(nullptr), file_start_time_ms_(0), fileNameManager_((Options*)options)
{
	// Nothing
}

FileOutput::~FileOutput()
{
	closeFile();
}

void FileOutput::outputBuffer(void *mem, size_t size, int64_t timestamp_us, uint32_t flags)
{
	if(options_->Get().force_dng) {
		saveDng(mem);
	} else {
		saveFile(mem, size, timestamp_us, flags);
	}

	bool isFirstFrame = getCurrentFileName().empty();
	std::string metadataFilename = options_->Get().output_metadata_location;
	libcamera::ControlList metadata;

	if(!options_->Get().metadata.empty() && !metadata_queue_.empty() && !metadataFilename.empty()) {
		metadata = metadata_queue_.front();
		json currentObject, metadata;
		metadata["filename"] = getCurrentFileName();
		metadata["metadata"] = metadata.toString();
		currentObject[std::to_string(fileNameManager_.getImagesWritten())] = metadata;
		if(isFirstFrame) {
			std::ofstream outFile(metadataFilename);
			if (!outFile.is_open())
				throw std::runtime_error("failed to open metadata output file " + metadataFilename);
			outFile << currentObject.dump();
			outFile.close();
		} else {
			std::ofstream outFile(metadataFilename, std::ios::app);
			if (!outFile.is_open())
				throw std::runtime_error("failed to open metadata output file " + metadataFilename);
			outFile << currentObject.dump();
			outFile.close();
		}
	}

}

void FileOutput::saveFile(void *mem, size_t size, int64_t timestamp_us, uint32_t flags) {
	// We need to open a new file if we're in "segment" mode and our segment is full
	// (though we have to wait for the next I frame), or if we're in "split" mode
	// and recording is being restarted (this is necessarily an I-frame already).
	if (fp_ == nullptr ||
		(options_->Get().segment && (flags & FLAG_KEYFRAME) &&
		 timestamp_us / 1000 - file_start_time_ms_ > options_->Get().segment) ||
		(options_->Get().split && (flags & FLAG_RESTART)))
	{
		closeFile();
		openFile(timestamp_us);
	}

	LOG(2, "FileOutput: output buffer " << mem << " size " << size);
	if (fp_ && size)
	{
		if (fwrite(mem, size, 1, fp_) != 1)
			throw std::runtime_error("failed to write output bytes");
		if (options_->Get().flush)
			fflush(fp_);
	}
}

void FileOutput::saveDng(void *mem) {
	libcamera::ControlList metadata;
	if (!options_->Get().metadata.empty())
	{
		metadata = metadata_queue_.front();
	} else {
		LOG(1, "No metadata");
	}
	std::string filename = fileNameManager_.getNextFileName();
	StreamInfo *info = this->getStreamInfo();

	dng_save(mem, *info, metadata, filename, "shadowgraph-v3", options_);
}

void FileOutput::openFile(int64_t timestamp_us)
{
	if (options_->Get().output == "-")
		fp_ = stdout;
	else if (!options_->Get().output.empty())
	{
		std::string filename = fileNameManager_.getNextFileName();
		fp_ = fopen(filename.c_str(), "w");
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
		if (options_->Get().flush)
			fflush(fp_);
		if (fp_ != stdout)
			fclose(fp_);
		fp_ = nullptr;
	}
}
