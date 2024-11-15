/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_raw.cpp - libcamera raw video record app.
 */

#include <chrono>

#include "core/rpicam_encoder.hpp"
#include "core/stream_info.hpp"
#include "encoder/null_encoder.hpp"
#include "output/output.hpp"

using namespace std::placeholders;

class LibcameraRaw : public RPiCamEncoder
{
public:
	LibcameraRaw() : RPiCamEncoder() {}

protected:
	// Force the use of "null" encoder.
	void createEncoder() { encoder_ = std::unique_ptr<Encoder>(new NullEncoder(GetOptions())); }
};

// The main even loop for the application.

static void event_loop(LibcameraRaw &app)
{
	VideoOptions const *options = app.GetOptions();
	std::unique_ptr<Output> output = std::unique_ptr<Output>(Output::Create(options));
	app.SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, output.get(), _1, _2, _3, _4));
	app.SetMetadataReadyCallback(std::bind(&Output::MetadataReady, output.get(), _1));

	app.OpenCamera();
	app.ConfigureVideo(LibcameraRaw::FLAG_VIDEO_RAW);
	app.StartEncoder();
	app.StartCamera();
	auto start_time = std::chrono::high_resolution_clock::now();

	for (unsigned int count = 0; ; count++)
	{
		LibcameraRaw::Msg msg = app.Wait();

		if (msg.type == RPiCamApp::MsgType::Timeout)
		{
			LOG_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
			app.StopCamera();
			app.StartCamera();
			continue;
		}
		if (msg.type != LibcameraRaw::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");
		if (count == 0)
		{
			StreamInfo info = app.GetStreamInfo(app.RawStream());
			output.get()->setStreamInfo(&info);
			libcamera::StreamConfiguration const &cfg = app.RawStream()->configuration();
			LOG(1, "Raw stream: " << cfg.size.width << "x" << cfg.size.height << " stride " << cfg.stride << " format "
								  << cfg.pixelFormat.toString());
		}

		LOG(2, "Viewfinder frame " << count);
		auto now = std::chrono::high_resolution_clock::now();
		if (options->timeout && (now - start_time) > options->timeout.value)
		{
			app.StopCamera();
			app.StopEncoder();
			return;
		}

		app.EncodeBuffer(std::get<CompletedRequestPtr>(msg.payload), app.RawStream());
	}
}

int main(int argc, char *argv[])
{
	try
	{
		LibcameraRaw app;
		VideoOptions *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			// Disable any codec (h.264/libav) based operations.
			options->codec = "yuv420";
			options->denoise = "cdn_off";
			options->nopreview = true;
			if (options->verbose >= 2)
				options->Print();

			event_loop(app);
		}
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
		return -1;
	}
	return 0;
}
