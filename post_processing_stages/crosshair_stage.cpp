/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * annotate_cv_stage.cpp - add text annotation to image
 */

// The text string can include the % directives supported by FrameInfo.

#include <time.h>

#include <libcamera/stream.h>

#include "core/rpicam_app.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;

using Stream = libcamera::Stream;

class CrosshairStage : public PostProcessingStage
{
public:
	CrosshairStage(RPiCamApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	Stream *stream_;
	StreamInfo info_;
	int line_thickness_;
};

#define NAME "crosshair"

char const *CrosshairStage::Name() const
{
	return NAME;
}

void CrosshairStage::Read(boost::property_tree::ptree const &params)
{
	line_thickness_ = params.get<int>("line_thickness", 2);
}

void CrosshairStage::Configure()
{
	stream_ = app_->GetMainStream();
	if (!stream_ || stream_->configuration().pixelFormat != libcamera::formats::YUV420)
		throw std::runtime_error("CrosshairStage: only YUV420 format supported");
	info_ = app_->GetStreamInfo(stream_);
}


bool CrosshairStage::Process(CompletedRequestPtr &completed_request)
{
	if (!stream_)
		return false;

	BufferWriteSync w(app_, completed_request->buffers[stream_]);
	libcamera::Span<uint8_t> buffer = w.Get()[0];
	uint32_t *ptr = (uint32_t *)buffer.data();
	StreamInfo info = app_->GetStreamInfo(stream_);

	Mat image(info.height, info.width, CV_8U, ptr, info.stride);

	Point center(info.width / 2, info.height / 2);
	line(image, center - Point(300, 0), center + Point(300, 0), Scalar(255, 255, 255), 2);
	line(image, center - Point(0, 300), center + Point(0, 300), Scalar(255, 255, 255), 2);

	return false;
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new CrosshairStage(app);
}

static RegisterStage reg(NAME, &Create);
