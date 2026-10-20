/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * shadowgraph_results.hpp - detection sidecar passed from the shadowgraph
 * stage to the file writer. The stage runs before the output filename exists,
 * so results are queued by the same timestamp the encoder later reports.
 */

#pragma once

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

struct ShadowgraphRoi
{
	int id;
	std::vector<uint8_t> png;
};

struct ShadowgraphSidecar
{
	std::string json;
	std::vector<ShadowgraphRoi> rois;
};

namespace ShadowgraphResults
{

void Publish(int64_t timestamp_us, ShadowgraphSidecar sidecar);
bool Take(int64_t timestamp_us, ShadowgraphSidecar &out);
void Discard(int64_t timestamp_us);
size_t Pending();

}

// Bare numbers are millimetres. A suffix of mm, in, or px selects the unit.
inline bool ParseShadowgraphMinSize(const std::string &text, double &value, std::string &unit, std::string &error)
{
	auto not_space = [](unsigned char c) { return !std::isspace(c); };
	auto begin = std::find_if(text.begin(), text.end(), not_space);
	auto end = std::find_if(text.rbegin(), text.rend(), not_space).base();
	if (begin >= end)
	{
		error = "empty minimum size";
		return false;
	}

	std::string s(begin, end);
	const char *start = s.c_str();
	char *stop = nullptr;
	value = std::strtod(start, &stop);
	if (stop == start)
	{
		error = "Unknown minimum size: " + text + ". Use a number, optionally followed by mm, in, or px.";
		return false;
	}

	std::string suffix(stop);
	auto suffix_begin = std::find_if(suffix.begin(), suffix.end(), not_space);
	auto suffix_end = std::find_if(suffix.rbegin(), suffix.rend(), not_space).base();
	suffix = suffix_begin < suffix_end ? std::string(suffix_begin, suffix_end) : std::string();
	for (char &c : suffix)
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

	if (suffix.empty())
		unit = "mm";
	else if (suffix == "mm" || suffix == "in" || suffix == "px")
		unit = suffix;
	else
	{
		error = "Unknown minimum size: " + text + ". Use a number, optionally followed by mm, in, or px.";
		return false;
	}

	if (!(value >= 0.0))
	{
		error = "Minimum size must be zero or positive";
		return false;
	}
	return true;
}
