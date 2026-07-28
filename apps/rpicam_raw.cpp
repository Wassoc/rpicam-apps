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

bool isAutoExposureEnabled(VideoOptions const *options)
{
	return !options->Get().shutter && !options->Get().gain;
}

static void enableAutoExposure(RPiCamApp &app)
{
	constexpr int64_t frame_time = 1000000 / 20; // 50,000 us = 20 fps
	libcamera::ControlList cl;
	cl.set(libcamera::controls::ExposureTimeMode, libcamera::controls::ExposureTimeModeAuto);
	cl.set(libcamera::controls::AnalogueGainMode, libcamera::controls::AnalogueGainModeAuto);
	cl.set(libcamera::controls::FrameDurationLimits,
		   libcamera::Span<const int64_t, 2>({ frame_time, frame_time }));
	app.SetControls(cl);
}

static void lockAutoExposure(RPiCamApp &app)
{
	int64_t frame_time = 1000000 / app.GetOptions()->Get().framerate.value_or(DEFAULT_FRAMERATE); // in us
	libcamera::ControlList cl;
	cl.set(libcamera::controls::ExposureTimeMode, libcamera::controls::ExposureTimeModeManual);
	cl.set(libcamera::controls::AnalogueGainMode, libcamera::controls::AnalogueGainModeManual);
	cl.set(libcamera::controls::FrameDurationLimits,
		   libcamera::Span<const int64_t, 2>({ frame_time, frame_time }));
	app.SetControls(cl);
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

// static void event_loop(LibcameraRaw &app, GpioHandler* lampHandler)
// {
// 	unsigned int AE_WARMUP_FRAMES = 10;
// 	unsigned int AE_WARMUP_CADENCE_SECONDS = 20;
// 	unsigned int ae_last_warmup_time = 0;
// 	unsigned int framesCaptured = 0;
// 	bool everyNthFrameEnabled = false;
// 	StreamInfo info;
// 	VideoOptions const *options = app.GetOptions();
// 	bool illuminationTriggerDisabled = options->Get().disable_illumination_trigger;
// 	bool autoExposureEnabled = isAutoExposureEnabled(options);
// 	std::unique_ptr<Output> output = std::unique_ptr<Output>(Output::Create(options));
// 	app.SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, output.get(), _1, _2, _3, _4));
// 	app.SetMetadataReadyCallback(std::bind(&Output::MetadataReady, output.get(), _1));

// 	if (options->Get().every_nth_frame > 1) {
// 		everyNthFrameEnabled = true;
// 	}
// 	if (lampHandler) {
// 		lampHandler->setNextLampColor();
// 	}
// 	app.OpenCamera();
// 	if (options->Get().force_jpeg) {
// 		app.ConfigureVideo(RPiCamEncoder::FLAG_VIDEO_JPEG_COLOURSPACE);
// 	} else if (options->Get().force_still) {
// 		app.ConfigureStill(RPiCamApp::FLAG_STILL_NONE);
// 	} else {
// 		app.ConfigureRawStream();
// 	}
// 	app.StartEncoder();
// 	app.StartCamera();
// 	auto start_time = std::chrono::high_resolution_clock::now();
// 	auto last_capture_time = start_time;
// 	libcamera::Stream *currentStream = nullptr;
// 	std::string currentStreamName = "";
// 	if (options->Get().force_jpeg) {
// 		currentStream = app.VideoStream();
// 		currentStreamName = "JPEG";
// 	} else if (options->Get().force_still) {
// 		currentStream = app.StillStream();
// 		currentStreamName = "STILL";
// 	} else {
// 		currentStream = app.RawStream();
// 		currentStreamName = "RAW";
// 	}

// 	// TODO: handle timelapses where the requested framerate is less than one a second
// 	for (long long count = -1; ; count++)
// 	{
// 		// Check for termination signals
// 		if (signal_received == SIGTERM || signal_received == SIGINT) {
// 			LOG(1, "Shutting down due to signal " << signal_received);
// 			app.StopCamera();
// 			app.StopEncoder();
// 			return;
// 		}
// 		LibcameraRaw::Msg msg = app.Wait();

// 		if (count == -1) {
// 			if(autoExposureEnabled) {
// 				ae_last_warmup_time = std::chrono::high_resolution_clock::now();
// 			}
// 			// Skip the first frame to allow the camera to warm up
// 			continue;
// 		}

// 		if (msg.type == RPiCamApp::MsgType::Timeout)
// 		{
// 			LOG_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
// 			app.StopCamera();
// 			app.StartCamera();
// 			continue;
// 		}
// 		if (msg.type != LibcameraRaw::MsgType::RequestComplete)
// 			throw std::runtime_error("unrecognised message!");
// 		if (count == 0)
// 		{
// 			info = app.GetStreamInfo(currentStream);
// 			output.get()->setStreamInfo(&info);
// 			libcamera::StreamConfiguration const &cfg = currentStream->configuration();
// 			LOG(1, currentStreamName << " stream: " << cfg.size.width << "x" << cfg.size.height << " stride " << cfg.stride << " format "
// 								  << cfg.pixelFormat.toString());
// 		}

// 		LOG(2, currentStreamName << " frame " << count);
// 		auto now = std::chrono::high_resolution_clock::now();
// 		if (options->Get().timeout && (now - start_time) > options->Get().timeout.value)
// 		{
// 			app.StopCamera();
// 			app.StopEncoder();
// 			return;
// 		}
// 		if (everyNthFrameEnabled) {
// 			long long every_nth_frame = (long long)options->Get().every_nth_frame;
// 			long long nth_frame = count % every_nth_frame;
// 			if (nth_frame == every_nth_frame - 1) {
// 				lampHandler->setNextLampColor();
// 				if (autoExposureEnabled) {
// 					enableAutoExposure(app);
// 				}
// 				if (illuminationTriggerDisabled) {
// 					lampHandler->turnOnLamp();
// 				}
// 			} else if (illuminationTriggerDisabled) {
// 				lampHandler->turnOffLamp();
// 			}
// 			if (nth_frame != 0) {
// 				continue;
// 			}
// 		} else if (options->Get().capture_interval && options->Get().capture_interval > 0.0f) {
// 			float time_since_last_capture = std::chrono::duration<float>(now - last_capture_time).count();
// 			if (time_since_last_capture >= options->Get().capture_interval) {
// 				last_capture_time = now;
// 			} else {
// 				continue;
// 			}
// 		}
// 		// Placing this after the interval check so we only update the lamp after the correct image has been captured
// 		CompletedRequestPtr completed_request = std::get<CompletedRequestPtr>(msg.payload);
// 		if (lampHandler) {
// 			std::string currentLampColor = lampHandler->getCurrentLampColor();
// 			if (everyNthFrameEnabled) {
// 				lampHandler->disableAllChannels();
// 				if (autoExposureEnabled) {
// 					lockAutoExposure(app);
// 				}
// 			} else {
// 				lampHandler->setNextLampColor();
// 			}
// 			completed_request->post_process_metadata.Set("exif_data.lamp_color", currentLampColor);
// 			completed_request->post_process_metadata.Set("exif_data.camera_serial_number", options->Get().camera_serial_number);
// 		}
// 		if (!app.EncodeBuffer(completed_request, currentStream))
// 		{
// 			// Keep advancing our "start time" if we're still waiting to start recording (e.g.
// 			// waiting for synchronisation with another camera).
// 			start_time = now;
// 		}
// 		framesCaptured++;
// 		if (options->Get().total_frames && framesCaptured == options->Get().total_frames) {
// 			app.StopCamera();
// 			app.StopEncoder();
// 			return;
// 		}
// 	}
// }

static void event_loop(LibcameraRaw &app, GpioHandler* lampHandler)
{
	unsigned int AE_WARMUP_FRAMES = 20;
	unsigned int AE_WARMUP_CADENCE_SECONDS = 20;
	auto ae_last_warmup_time = std::chrono::high_resolution_clock::now();
	float time_since_last_ae_warmup = 0.0f;
	unsigned int ae_warmup_frames_captured = 0;
	bool ae_warmup_in_progress = false;
	unsigned int requests_ignored_per_capture = 0;
	unsigned int requests_ignored_since_last_capture = 0;
	unsigned int framesCaptured = 0;
	bool everyNthFrameEnabled = false;
	StreamInfo info;
	VideoOptions const *options = app.GetOptions();
	bool illuminationTriggerDisabled = options->Get().disable_illumination_trigger;
	bool autoExposureEnabled = isAutoExposureEnabled(options);
	std::unique_ptr<Output> output = std::unique_ptr<Output>(Output::Create(options));
	app.SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, output.get(), _1, _2, _3, _4));
	app.SetMetadataReadyCallback(std::bind(&Output::MetadataReady, output.get(), _1));

	if (options->Get().every_nth_frame > 1) {
		everyNthFrameEnabled = true;
		requests_ignored_per_capture = options->Get().every_nth_frame - 1;
	}
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

		if (autoExposureEnabled) {
			if (ae_warmup_in_progress) {
				ae_warmup_frames_captured++;
				if (AE_WARMUP_FRAMES - ae_warmup_frames_captured == 3) {
					lockAutoExposure(app);
				}
				if (ae_warmup_frames_captured >= AE_WARMUP_FRAMES) {
					ae_warmup_in_progress = false;
					ae_warmup_frames_captured = 0;
					if (lampHandler) {
						lampHandler->setNextLampColor();
					}
				}
				continue;
			}
			time_since_last_ae_warmup = std::chrono::duration<float>(now - ae_last_warmup_time).count();
			if (time_since_last_ae_warmup >= AE_WARMUP_CADENCE_SECONDS) {
				ae_last_warmup_time = now;
				ae_warmup_in_progress = true;
				enableAutoExposure(app);
				if (lampHandler) {
					lampHandler->enableStrobe();
				}
			}
			// We are not in the AE warmup phase, so we can capture an image
		}
		if (requests_ignored_per_capture > 0 && requests_ignored_since_last_capture < requests_ignored_per_capture) {
			// Next completed request will be saved to disk
			if (requests_ignored_per_capture - requests_ignored_since_last_capture == 1) {
				lampHandler->setNextLampColor();
				if (illuminationTriggerDisabled) {
					lampHandler->turnOnLamp();
				}
			} else if (illuminationTriggerDisabled) {
				lampHandler->turnOffLamp();
			}
			requests_ignored_since_last_capture++;
			continue;
		}
		// At this point, we are writing the latest request to disk
		requests_ignored_since_last_capture = 0;
		CompletedRequestPtr completed_request = std::get<CompletedRequestPtr>(msg.payload);
		if (lampHandler) {
			std::string currentLampColor = lampHandler->getCurrentLampColor();
			if (everyNthFrameEnabled) {
				lampHandler->disableAllChannels();
				if (illuminationTriggerDisabled) {
					lampHandler->turnOffLamp();
				}
			} else {
				lampHandler->setNextLampColor();
			}
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
				unsigned int brightness_zero = options->Get().r_brightness;
				unsigned int brightness_one = options->Get().g_brightness;
				unsigned int brightness_two = options->Get().b_brightness;
				if (options->Get().lamp_pattern.find("S") != std::string::npos || options->Get().lamp_pattern.find("s") != std::string::npos) {
					brightness_one = options->Get().s_brightness;
				}
				lampHandler = new GpioHandler(options->Get().lamp_pattern, brightness_zero, brightness_one, brightness_two, options->Get().disable_illumination_trigger, options->Get().fire_and_forget);
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

