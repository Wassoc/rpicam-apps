/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_raw.cpp - libcamera raw video record app.
 */

#include <chrono>
#include <memory>
#include <signal.h>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "core/rpicam_encoder.hpp"
#include "core/shadowgraph_results.hpp"
#include "core/stream_info.hpp"
#include "post_processing_stages/post_processing_stage.hpp"
#include "encoder/null_encoder.hpp"
#include "encoder/mjpeg_encoder.hpp"
#include "encoder/png_encoder.hpp"
#include "encoder/dng_encoder.hpp"
#include "output/output.hpp"
#include "wassoc-utils/gpiohandler.hpp"

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

static int64_t frameTimestampUs(const CompletedRequestPtr &completed_request, libcamera::Stream *stream)
{
	auto ts = completed_request->metadata.get(controls::FrameWallClock);
	return ts ? *ts : completed_request->buffers[stream]->metadata().timestamp / 1000;
}

static void discardShadowgraph(const CompletedRequestPtr &completed_request, libcamera::Stream *stream)
{
	ShadowgraphResults::Discard(frameTimestampUs(completed_request, stream));
}

static void event_loop(LibcameraRaw &app, GpioHandler* lampHandler)
{
	unsigned int framesCaptured = 0;
	bool everyNthFrameEnabled = false;
	StreamInfo info;
	VideoOptions const *options = app.GetOptions();
	std::unique_ptr<Output> output = std::unique_ptr<Output>(Output::Create(options));
	app.SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, output.get(), std::placeholders::_1,
											 std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	app.SetMetadataReadyCallback(std::bind(&Output::MetadataReady, output.get(), std::placeholders::_1));

	if (options->Get().every_nth_frame > 1) {
		everyNthFrameEnabled = true;
	}
	if (lampHandler) {
		lampHandler->setNextLampColor();
	}
	app.OpenCamera();
	std::unique_ptr<PostProcessingStage> shadowgraph;
	if (options->Get().shadowgraph_detect && !app.HasPostProcessStage("shadowgraph_detect_cv"))
	{
		PostProcessingStage *stage = app.MakePostProcessStage("shadowgraph_detect_cv");
		if (!stage)
			LOG_ERROR("shadowgraph_detect_cv is not available; rebuild rpicam-apps with OpenCV enabled");
		else
		{
			boost::property_tree::ptree params;
			if (!options->Get().post_process_file.empty())
			{
				boost::property_tree::ptree root;
				boost::property_tree::read_json(options->Get().post_process_file, root);
				auto it = root.find("shadowgraph_detect_cv");
				if (it != root.not_found())
					params = it->second;
			}
			stage->Read(params);
			shadowgraph.reset(stage);
			LOG(1, "Shadowgraph detect enabled");
		}
	}
	else if (options->Get().shadowgraph_detect)
		LOG(1, "Shadowgraph detect enabled from the post-process file");
	if (options->Get().force_jpeg) {
		app.ConfigureVideo(RPiCamEncoder::FLAG_VIDEO_JPEG_COLOURSPACE);
	} else if (options->Get().force_still) {
		app.ConfigureStill(RPiCamApp::FLAG_STILL_NONE);
	} else {
		app.ConfigureRawStream();
	}
	if (shadowgraph)
		shadowgraph->Configure();
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
			// Skip the first frame to allow the camera to warm up.
			// A post-process-file stage may already have published a result.
			if (msg.type == LibcameraRaw::MsgType::RequestComplete)
				discardShadowgraph(std::get<CompletedRequestPtr>(msg.payload), currentStream);
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
			if (msg.type == LibcameraRaw::MsgType::RequestComplete)
				discardShadowgraph(std::get<CompletedRequestPtr>(msg.payload), currentStream);
			app.StopCamera();
			app.StopEncoder();
			return;
		}
		if (everyNthFrameEnabled) {
			long long every_nth_frame = (long long)options->Get().every_nth_frame;
			long long nth_frame = count % every_nth_frame;
			if (nth_frame == every_nth_frame - 1) {
				lampHandler->setNextLampColor();
			}
			if (nth_frame != 0) {
				discardShadowgraph(std::get<CompletedRequestPtr>(msg.payload), currentStream);
				continue;
			}
		} else if (options->Get().capture_interval && options->Get().capture_interval > 0.0f) {
			float time_since_last_capture = std::chrono::duration<float>(now - last_capture_time).count();
			if (time_since_last_capture >= options->Get().capture_interval) {
				last_capture_time = now;
			} else {
				discardShadowgraph(std::get<CompletedRequestPtr>(msg.payload), currentStream);
				continue;
			}
		}
		// Placing this after the interval check so we only update the lamp after the correct image has been captured
		CompletedRequestPtr completed_request = std::get<CompletedRequestPtr>(msg.payload);
		if (lampHandler) {
			std::string currentLampColor = lampHandler->getCurrentLampColor();
			if (everyNthFrameEnabled) {
				lampHandler->disableAllChannels();
				// Detection and PNG encode can block long enough that the next
				// saved frame is already exposed before setNextLampColor runs.
				// Restore this color now; the pattern still advances on the
				// skipped frame immediately before the next save.
				if (shadowgraph)
					lampHandler->enableCurrentChannels();
			} else {
				lampHandler->setNextLampColor();
			}
			completed_request->post_process_metadata.Set("exif_data.lamp_color", currentLampColor);
			completed_request->post_process_metadata.Set("exif_data.camera_serial_number", options->Get().camera_serial_number);
		}
		if (shadowgraph)
			shadowgraph->Process(completed_request);
		if (!app.EncodeBuffer(completed_request, currentStream))
		{
			// Keep advancing our "start time" if we're still waiting to start recording (e.g.
			// waiting for synchronisation with another camera).
			discardShadowgraph(completed_request, currentStream);
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

