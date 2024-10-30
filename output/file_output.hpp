/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * file_output.hpp - Write output to file.
 */

#pragma once

#include <filesystem>
#include "output.hpp"

namespace fs = std::filesystem;

class FileOutput : public Output
{
public:
	FileOutput(VideoOptions const *options);
	~FileOutput();

protected:
	void outputBuffer(void *mem, size_t size, int64_t timestamp_us, uint32_t flags) override;

private:
	void openFile(int64_t timestamp_us);
	void closeFile();
	void makeNewCurrentDir();
	unsigned int getDirectorySize(const fs::path& dirPath);
	std::string getOutputDirectoryPrefix();
	std::string getSubstringAfterPrefix(const std::string& str, const std::string& prefix);
	void initializeCurrentOperatingDirectory();
	FILE *fp_;
	unsigned int count_;
	unsigned int directory_count_;
	fs::path current_directory_;
	int64_t file_start_time_ms_;
};
