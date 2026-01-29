/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_raw.cpp - libcamera raw video record app.
 */

#include <chrono>
#include <signal.h>

#include "core/rpicam_encoder.hpp"
#include "core/stream_info.hpp"
#include "encoder/null_encoder.hpp"
#include "encoder/mjpeg_encoder.hpp"
#include "encoder/png_encoder.hpp"
#include "encoder/dng_encoder.hpp"
#include "output/output.hpp"
#include "wassoc-utils/gpiohandler.hpp"


using namespace std::placeholders;

// Signal handling
static volatile int signal_received = 0;

static void signal_handler(int signal_number)
{
	signal_received = signal_number;
	LOG(1, "Received signal " << signal_number);
}

class LibcameraRaw : public RPiCamEncoder
{
public:
	LibcameraRaw() : RPiCamEncoder() {}
protected:
	// Force the use of "null" encoder.
	void createEncoder() {
		if (GetOptions()->Get().force_png) {
			encoder_ = std::unique_ptr<Encoder>(new PngEncoder(GetOptions()));
		} else if (GetOptions()->Get().force_jpeg || GetOptions()->Get().force_still) {
			encoder_ = std::unique_ptr<Encoder>(new MjpegEncoder(GetOptions()));
		} else {
			encoder_ = std::unique_ptr<Encoder>(new DngEncoder(GetOptions()));
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

	if (lampHandler) {
		lampHandler->setNextLampColor();
	}
	app.OpenCamera();
	if (options->Get().force_jpeg) {
		app.ConfigureVideo(RPiCamEncoder::FLAG_VIDEO_JPEG_COLOURSPACE);
	} else if (options->Get().force_still) {
		app.ConfigureStill(RPiCamApp::FLAG_STILL_NONE);
	} else {
		app.ConfigureRawStream();
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
	for (long long count = -1; ; count++)
	{
		// Check for termination signals
		if (signal_received == SIGTERM || signal_received == SIGINT) {
			LOG(1, "Shutting down due to signal " << signal_received);
			app.StopCamera();
			app.StopEncoder();
			return;
		}
		LibcameraRaw::Msg msg = app.Wait();

		if (count == -1) {
			// Skip the first frame to allow the camera to warm up
			continue;
		}

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
		if (options->Get().capture_interval && options->Get().capture_interval > 0.0f) {
			float time_since_last_capture = std::chrono::duration<float>(now - last_capture_time).count();
			if (time_since_last_capture >= options->Get().capture_interval) {
				last_capture_time = now;
			} else {
				continue;
			}
		} else if (options->Get().every_nth_frame > 1 && count % (long long)options->Get().every_nth_frame != 0) {
			continue;
		}
		// Placing this after the interval check so we only update the lamp after the correct image has been captured
		CompletedRequestPtr completed_request = std::get<CompletedRequestPtr>(msg.payload);
		if (lampHandler) {
			std::string currentLampColor = lampHandler->getCurrentLampColor();
			// lampHandler->setNextLampColor();
			completed_request->post_process_metadata.Set("exif_data.lamp_color", currentLampColor);
			completed_request->post_process_metadata.Set("exif_data.camera_serial_number", options->Get().camera_serial_number);
		}
		if (!app.EncodeBuffer(completed_request, currentStream))
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
			// Register signal handlers for graceful shutdown
			signal(SIGTERM, signal_handler);
			signal(SIGINT, signal_handler);
			GpioHandler* lampHandler = nullptr;
			if (!options->Get().without_lamp) {
				lampHandler = new GpioHandler(options->Get().lamp_pattern, options->Get().r_brightness, options->Get().g_brightness, options->Get().b_brightness, options->Get().disable_illumination_trigger, options->Get().fire_and_forget);
			}
			// Disable any codec (h.264/libav) based operations.
			options->Set().codec = "yuv420";
			options->Set().denoise = "cdn_off";
			options->Set().nopreview = true;
			if (options->Get().verbose >= 2)
				options->Get().Print();

			event_loop(app, lampHandler);
			if (lampHandler) {
				delete lampHandler;
			}
		}
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
		return -1;
	}
	return 0;
}

