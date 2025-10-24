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
#include "encoder/mjpeg_encoder.hpp"
#include "output/output.hpp"
#include "gpiohandler/gpiohandler.hpp"


using namespace std::placeholders;

class LibcameraRaw : public RPiCamEncoder
{
public:
	LibcameraRaw() : RPiCamEncoder() {}
protected:
	// Force the use of "null" encoder.
	void createEncoder() {
		if (GetOptions()->Get().force_jpeg || GetOptions()->Get().force_still) {
			encoder_ = std::unique_ptr<Encoder>(new MjpegEncoder(GetOptions()));
		} else {
			encoder_ = std::unique_ptr<Encoder>(new NullEncoder(GetOptions()));
		}
	}
};

// The main even loop for the application.

static void event_loop(LibcameraRaw &app, GpioHandler* lampHandler)
{
	unsigned int framesCaptured = 0;
	StreamInfo info;
	VideoOptions const *options = app.GetOptions();
	std::unique_ptr<Output> output = std::unique_ptr<Output>(Output::Create(options));
	app.SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, output.get(), _1, _2, _3, _4));
	app.SetMetadataReadyCallback(std::bind(&Output::MetadataReady, output.get(), _1));

	lampHandler->setNextLampColor();
	app.OpenCamera();
	if (options->Get().force_jpeg) {
		app.ConfigureVideo(RPiCamEncoder::FLAG_VIDEO_JPEG_COLOURSPACE);
	} else if (options->Get().force_still) {
		app.ConfigureStill(RPiCamApp::FLAG_STILL_NONE);
	} else {
		// app.ConfigureRawStream();
		app.ConfigureStill(RPiCamApp::FLAG_STILL_RAW);
	}
	app.StartEncoder();
	app.StartCamera();
	auto start_time = std::chrono::high_resolution_clock::now();
	auto last_capture_time = start_time;
	libcamera::Stream *currentStream = nullptr;
	std::string currentStreamName = "";
	if (options->Get().force_jpeg) {
		currentStream = app.VideoStream();
		currentStreamName = "JPEG";
	} else if (options->Get().force_still) {
		currentStream = app.StillStream();
		currentStreamName = "STILL";
	} else {
		currentStream = app.RawStream();
		currentStreamName = "RAW";
	}

	// TODO: handle timelapses where the requested framerate is less than one a second
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
			info = app.GetStreamInfo(currentStream);
			output.get()->setStreamInfo(&info);
			libcamera::StreamConfiguration const &cfg = currentStream->configuration();
			LOG(1, currentStreamName << " stream: " << cfg.size.width << "x" << cfg.size.height << " stride " << cfg.stride << " format "
								  << cfg.pixelFormat.toString());
		}

		LOG(2, currentStreamName << " frame " << count);
		auto now = std::chrono::high_resolution_clock::now();
		if (options->Get().timeout && (now - start_time) > options->Get().timeout.value)
		{
			app.StopCamera();
			app.StopEncoder();
			return;
		}
		if (options->Get().capture_interval && options->Get().capture_interval > 0) {
			unsigned int time_since_last_capture = std::chrono::duration_cast<std::chrono::seconds>(now - last_capture_time).count();
			if (time_since_last_capture >= options->Get().capture_interval) {
				last_capture_time = now;
			} else {
				continue;
			}
		}
		// Placing this after the interval check so we only update the lamp after the correct image has been captured
		lampHandler->setNextLampColor();
		if (!app.EncodeBuffer(std::get<CompletedRequestPtr>(msg.payload), currentStream))
		{
			// Keep advancing our "start time" if we're still waiting to start recording (e.g.
			// waiting for synchronisation with another camera).
			start_time = now;
		}
		framesCaptured++;
		if (options->Get().total_frames && framesCaptured == options->Get().total_frames) {
			app.StopCamera();
			app.StopEncoder();
			return;
		}
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
			GpioHandler* lampHandler = new GpioHandler(options->Get().lamp_pattern);
			// Disable any codec (h.264/libav) based operations.
			options->Set().codec = "yuv420";
			options->Set().denoise = "cdn_off";
			options->Set().nopreview = true;
			if (options->Get().verbose >= 2)
				options->Get().Print();

			event_loop(app, lampHandler);
			delete lampHandler;
		}
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
		return -1;
	}
	return 0;
}
