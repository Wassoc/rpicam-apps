/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * file_output.cpp - Write output to file.
 */
#include <filesystem>
#include <string>
#include <fstream>
#include <mutex>
#include <cctype>

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

namespace
{
std::mutex g_metadata_file_mutex;

static std::streamoff findLastNonWhitespacePos(std::fstream &f, std::streamoff startPosInclusive)
{
	for (std::streamoff pos = startPosInclusive; pos >= 0; --pos)
	{
		f.clear();
		f.seekg(pos, std::ios::beg);
		char c = '\0';
		if (!f.get(c))
			continue;
		if (!std::isspace(static_cast<unsigned char>(c)))
			return pos;
	}
	return -1;
}

static void resetMetadataFile(std::string const &metadataFilename)
{
	std::ofstream out(metadataFilename, std::ios::out | std::ios::trunc | std::ios::binary);
	if (!out.is_open())
		throw std::runtime_error("failed to open metadata output file " + metadataFilename);
	out << "{\n}\n";
}

static void appendMetadataEntry(std::string const &metadataFilename, std::string const &key, json const &metadataJson)
{
	// Keep the file as a single JSON object and append new entries by seeking to the final '}'.
	// This avoids reading/parsing the whole file on every frame.
	if (!fs::exists(metadataFilename) || fs::file_size(metadataFilename) == 0)
		resetMetadataFile(metadataFilename);

	std::fstream f(metadataFilename, std::ios::in | std::ios::out | std::ios::binary);
	if (!f.is_open())
		throw std::runtime_error("failed to open metadata output file " + metadataFilename);

	std::streamoff fileSize = static_cast<std::streamoff>(fs::file_size(metadataFilename));
	std::streamoff closeBracePos = findLastNonWhitespacePos(f, fileSize > 0 ? fileSize - 1 : 0);
	if (closeBracePos < 0)
	{
		// Corrupt/empty file; reset and retry.
		f.close();
		resetMetadataFile(metadataFilename);
		f.open(metadataFilename, std::ios::in | std::ios::out | std::ios::binary);
		fileSize = static_cast<std::streamoff>(fs::file_size(metadataFilename));
		closeBracePos = findLastNonWhitespacePos(f, fileSize - 1);
	}

	// Ensure the last non-whitespace character is a closing brace.
	f.clear();
	f.seekg(closeBracePos, std::ios::beg);
	char lastChar = '\0';
	f.get(lastChar);
	if (lastChar != '}')
	{
		f.close();
		resetMetadataFile(metadataFilename);
		f.open(metadataFilename, std::ios::in | std::ios::out | std::ios::binary);
		fileSize = static_cast<std::streamoff>(fs::file_size(metadataFilename));
		closeBracePos = findLastNonWhitespacePos(f, fileSize - 1);
		f.clear();
		f.seekg(closeBracePos, std::ios::beg);
		f.get(lastChar);
	}

	// Detect whether the object is currently empty: "{ ... }" where the previous non-whitespace is '{'.
	std::streamoff prevPos = findLastNonWhitespacePos(f, closeBracePos - 1);
	bool isEmptyObject = false;
	if (prevPos >= 0)
	{
		f.clear();
		f.seekg(prevPos, std::ios::beg);
		char prevChar = '\0';
		if (f.get(prevChar) && prevChar == '{')
			isEmptyObject = true;
	}

	std::string entry = "  \"" + key + "\": " + metadataJson.dump(2);
	std::string insertion = (isEmptyObject ? "\n" : ",\n") + entry + "\n}\n";

	// Overwrite the final '}' with our insertion, then truncate any leftover bytes.
	f.clear();
	f.seekp(closeBracePos, std::ios::beg);
	f.write(insertion.data(), static_cast<std::streamsize>(insertion.size()));
	f.flush();

	auto newEnd = f.tellp();
	f.close();

	if (newEnd != std::streampos(-1))
		fs::resize_file(metadataFilename, static_cast<uintmax_t>(newEnd));
}
} // namespace

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
	saveFile(mem, size, timestamp_us, flags);

	bool isFirstFrame = fileNameManager_.getImagesWritten() == 1;
	std::string metadataFilename = options_->Get().output_metadata_location;
	libcamera::ControlList metadata;

	if(!options_->Get().metadata.empty() && !metadata_queue_.empty() && !metadataFilename.empty()) {
		metadata = metadata_queue_.front();
		const libcamera::ControlIdMap *id_map = metadata.idMap();
		json metadataJson, metadataSummary;
		metadataJson["filename"] = getCurrentFileName();
		for (auto const &[id, val] : metadata)
			metadataSummary[id_map->at(id)->name()] = val.toString();
		metadataJson["metadata"] = metadataSummary;

		std::lock_guard<std::mutex> lock(g_metadata_file_mutex);
		if (isFirstFrame)
			resetMetadataFile(metadataFilename);
		appendMetadataEntry(metadataFilename, std::to_string(fileNameManager_.getImagesWritten() - 1), metadataJson);
	} else {
		LOG(1, "No Metadata found");
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

void FileOutput::savePng(void *mem) {
	std::string filename = fileNameManager_.getNextFileName();
	StreamInfo *info = this->getStreamInfo();
	png_save(mem, *info, filename);
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
