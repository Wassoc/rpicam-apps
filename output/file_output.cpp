/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * file_output.cpp - Write output to file.
 */
#include <filesystem>
#include <string>

#include "file_output.hpp"

namespace fs = std::filesystem;

FileOutput::FileOutput(VideoOptions const *options)
	: Output(options), fp_(nullptr), count_(0), file_start_time_ms_(0)
{
	initializeCurrentOperatingDirectory();
}

FileOutput::~FileOutput()
{
	closeFile();
}

void FileOutput::outputBuffer(void *mem, size_t size, int64_t timestamp_us, uint32_t flags)
{
	if(current_directory_size_ > options_->max_directory_size) {
		makeNewCurrentDir();
	}
	// We need to open a new file if we're in "segment" mode and our segment is full
	// (though we have to wait for the next I frame), or if we're in "split" mode
	// and recording is being restarted (this is necessarily an I-frame already).
	if (fp_ == nullptr ||
		(options_->segment && (flags & FLAG_KEYFRAME) &&
		 timestamp_us / 1000 - file_start_time_ms_ > options_->segment) ||
		(options_->split && (flags & FLAG_RESTART)))
	{
		closeFile();
		openFile(timestamp_us);
	}

	LOG(2, "FileOutput: output buffer " << mem << " size " << size);
	if (fp_ && size)
	{
		if (fwrite(mem, size, 1, fp_) != 1)
			throw std::runtime_error("failed to write output bytes");
		if (options_->flush)
			fflush(fp_);
	}
	current_directory_size_++;
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

void FileOutput::makeNewCurrentDir() {
	directory_count_++;

    try {
		char newDirName[256];
		snprintf(newDirName, sizeof(newDirName), options_->output_directory.c_str(), directory_count_);
		fs::path newOperatingDir = fs::path(options_->parent_directory) / std::string(newDirName);
        // Create the directory
        if (fs::create_directory(newOperatingDir)) {
            std::cout << "Directory created: " << newOperatingDir << std::endl;
			current_directory_size_ = 0;
			current_directory_ = newOperatingDir;
        } else {
            std::cout << "Directory already exists: " << newOperatingDir << std::endl;
        }
    } catch (const fs::filesystem_error& e) {
        std::cerr << "Error creating directory: " << e.what() << std::endl;
    }
}

unsigned int FileOutput::getDirectorySize(const fs::path& dirPath) {
    unsigned int totalSize = 0;

    if (fs::exists(dirPath) && fs::is_directory(dirPath)) {
        for (const auto& entry : fs::recursive_directory_iterator(dirPath)) {
            if (fs::is_regular_file(entry)) {
                totalSize += 1;
            }
        }
    } else {
        std::cerr << "Invalid directory path." << std::endl;
		totalSize = -1;
    }

    return totalSize;
}

std::string FileOutput::getOutputDirectoryPrefix() {
	if(options_->output_directory == "") {
		return "";
	}
    // Find the position of the '%' character
    size_t pos = options_->output_directory.find('%');
    
    // If '%' is found, return the substring up to that position
    if (pos != std::string::npos) {
        return options_->output_directory.substr(0, pos);
    }
    
    // If '%' is not found, return the whole string
    return options_->output_directory;
}

std::string FileOutput::getSubstringAfterPrefix(const std::string& str, const std::string& prefix) {
    // Find the position of the prefix
    size_t pos = str.find(prefix);
    
    // If the prefix is found, return the substring after it
    if (pos != std::string::npos) {
        return str.substr(pos + prefix.length());
    }
    
    // If the prefix is not found, return an empty string or handle as needed
    return "";
}



void FileOutput::initializeCurrentOperatingDirectory() {
	fs::path parentDir = options_->parent_directory;
	std::string outputDirPrefix = getOutputDirectoryPrefix();
	std::string outputDirWithHighestNumber = "default";
	int maxNum = 0;

    if (fs::exists(parentDir) && fs::is_directory(parentDir)) {
		// find the directory with the highest value
		// ex. a parent dir with Dir0001, Dir0002 ... Dir7777 will select Dir 7777
        for (const auto& curDir : fs::directory_iterator(parentDir)) {
            if (fs::is_directory(curDir)) {
                std::string dirName = curDir.path().filename().string();
                // Check to see if the current dir matches the prefix supplied by the options_->output_directory
				// Example. Dir%05d should match all directories with the "Dir" prefix
				// Possible TODO: Guard against directories with similar prefixes, Dir%05d would end up matching a directory with the name "DirectoriesAreAwesome9876"
                if (dirName.rfind(outputDirPrefix, 0) == 0) {
					// For every directory that matches the prefix, get the number postfix. ex. Dir9876 would return 9876
					// Check to see if the current dir has the highest number so far
                    int dirNum = std::stoi(getSubstringAfterPrefix(dirName, outputDirPrefix));
					if(dirNum >= maxNum) {
						maxNum = dirNum;
						outputDirWithHighestNumber = dirName;
					}
                }
            }
        }

		// Now that we have the directory with the highest value, check to see if there is space in that dir
		fs::path outputDirectoryPath = fs::path(options_->parent_directory) / outputDirWithHighestNumber;
		unsigned int dirSize = getDirectorySize(outputDirectoryPath);
		if(dirSize < options_->max_directory_size) {
			current_directory_ = outputDirectoryPath;
			current_directory_size_ = dirSize;
			directory_count_ = maxNum;
		} else {
			// Not enough space in the current Dir, make a new one
			makeNewCurrentDir();
		}

    } else {
        std::cerr << "Invalid directory path." << std::endl;
    }
}
