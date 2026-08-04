#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>

#include "core/options.hpp"

namespace fs = std::filesystem;

// Builds output paths for still/raw capture and video recording.
//
// Two modes:
// - Simple: -o is used as-is (optional printf-style %d counter). Used when
//   --parent-directory / --output-directory are not both set.
// - Managed: files go under parent_directory / output_directory%N /, and a new
//   numbered directory is created when max_directory_size is reached.
class FileNameManager
{
public:
	explicit FileNameManager(Options const *options)
		: options_(options), directory_count_(0), current_directory_size_(0), files_written_(0),
		  managed_(isManagedNaming(options))
	{
		if (managed_)
			initializeCurrentOperatingDirectory();
	}

	static bool isManagedNaming(Options const *options)
	{
		return options && !options->Get().parent_directory.empty() && !options->Get().output_directory.empty();
	}

	// True for paths that should be resolved by FileNameManager (not pipes/URLs).
	static bool isLocalFileOutput(std::string const &output)
	{
		if (output.empty() || output == "-")
			return false;
		if (output.compare(0, 6, "tcp://") == 0 || output.compare(0, 6, "udp://") == 0)
			return false;
		return true;
	}

	std::string getNextFileName()
	{
		if (options_->Get().output.empty())
			throw std::runtime_error("output file name required");

		// max_directory_size == 0 means unlimited (do not rotate).
		if (managed_ && options_->Get().max_directory_size > 0 &&
			current_directory_size_ >= options_->Get().max_directory_size)
			makeNewCurrentDir();

		char filename[256];
		int n = snprintf(filename, sizeof(filename), options_->Get().output.c_str(), files_written_);
		if (n < 0 || static_cast<size_t>(n) >= sizeof(filename))
			throw std::runtime_error("failed to generate filename");

		fs::path pathToFile;
		if (managed_)
		{
			if (current_directory_.empty())
				throw std::runtime_error("no output directory available");
			// current_directory_ is already the full path to the operating directory.
			pathToFile = current_directory_ / filename;
		}
		else
		{
			pathToFile = filename;
		}

		if (options_->Get().force_dng && pathToFile.extension() != DNG_EXTENSION)
			pathToFile.replace_extension(DNG_EXTENSION);

		files_written_++;
		if (managed_)
			current_directory_size_++;
		current_file_name_ = pathToFile.string();
		return current_file_name_;
	}

	std::string getCurrentFileName() const { return current_file_name_; }

	unsigned int getFilesWritten() const { return files_written_; }

	// Legacy name used by still/raw capture paths.
	unsigned int getImagesWritten() const { return files_written_; }

private:
	inline static const std::string DNG_EXTENSION = ".dng";

	Options const *options_;
	unsigned int directory_count_;
	unsigned int current_directory_size_;
	unsigned int files_written_;
	bool managed_;
	fs::path current_directory_;
	std::string current_file_name_;

	void makeNewCurrentDir()
	{
		directory_count_++;

		char newDirName[256];
		int n = snprintf(newDirName, sizeof(newDirName), options_->Get().output_directory.c_str(), directory_count_);
		if (n < 0 || static_cast<size_t>(n) >= sizeof(newDirName))
			throw std::runtime_error("failed to generate output directory name");

		fs::path newOperatingDir = fs::path(options_->Get().parent_directory) / newDirName;
		try
		{
			if (!fs::exists(newOperatingDir))
				fs::create_directories(newOperatingDir);
			else if (!fs::is_directory(newOperatingDir))
				throw std::runtime_error("output path exists and is not a directory: " + newOperatingDir.string());

			current_directory_size_ = getDirectorySize(newOperatingDir);
			current_directory_ = newOperatingDir;
		}
		catch (const fs::filesystem_error &e)
		{
			throw std::runtime_error(std::string("failed to create output directory: ") + e.what());
		}
	}

	unsigned int getDirectorySize(const fs::path &dirPath) const
	{
		unsigned int totalSize = 0;

		if (!fs::exists(dirPath) || !fs::is_directory(dirPath))
			return 0;

		for (const auto &entry : fs::directory_iterator(dirPath))
		{
			if (fs::is_regular_file(entry))
				totalSize++;
		}

		return totalSize;
	}

	std::string getOutputDirectoryPrefix() const
	{
		std::string const &pattern = options_->Get().output_directory;
		size_t pos = pattern.find('%');
		if (pos != std::string::npos)
			return pattern.substr(0, pos);
		return pattern;
	}

	static std::string getSubstringAfterPrefix(const std::string &str, const std::string &prefix)
	{
		if (str.rfind(prefix, 0) != 0)
			return "";
		return str.substr(prefix.length());
	}

	void initializeCurrentOperatingDirectory()
	{
		fs::path parentDir = options_->Get().parent_directory;
		if (!fs::exists(parentDir))
			fs::create_directories(parentDir);
		if (!fs::is_directory(parentDir))
			throw std::runtime_error("parent directory is not a directory: " + parentDir.string());

		std::string outputDirPrefix = getOutputDirectoryPrefix();
		std::string outputDirWithHighestNumber;
		int maxNum = 0;
		bool found = false;

		for (const auto &curDir : fs::directory_iterator(parentDir))
		{
			if (!fs::is_directory(curDir))
				continue;

			std::string dirName = curDir.path().filename().string();
			// Match directories that start with the output-directory prefix (e.g. "Dir" for "Dir%05d").
			// Note: "Dir%05d" will also match names like "DirectoriesAreAwesome9876".
			if (dirName.rfind(outputDirPrefix, 0) != 0)
				continue;

			std::string suffix = getSubstringAfterPrefix(dirName, outputDirPrefix);
			if (suffix.empty())
				continue;

			try
			{
				int dirNum = std::stoi(suffix);
				if (!found || dirNum >= maxNum)
				{
					maxNum = dirNum;
					outputDirWithHighestNumber = dirName;
					found = true;
				}
			}
			catch (const std::exception &)
			{
				continue;
			}
		}

		directory_count_ = maxNum;
		if (found)
		{
			fs::path outputDirectoryPath = parentDir / outputDirWithHighestNumber;
			unsigned int dirSize = getDirectorySize(outputDirectoryPath);
			if (options_->Get().max_directory_size == 0 || dirSize < options_->Get().max_directory_size)
			{
				current_directory_ = outputDirectoryPath;
				current_directory_size_ = dirSize;
				return;
			}
		}

		makeNewCurrentDir();
	}
};
