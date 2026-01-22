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

using Stream = libcamera::Stream;

class PopulateExifDataStage : public PostProcessingStage
{
public:
	PopulateExifDataStage(RPiCamApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	Stream *stream_;
	StreamInfo info_;
};

#define NAME "populate_exif_data"

char const *PopulateExifDataStage::Name() const
{
	return NAME;
}

void PopulateExifDataStage::Read(boost::property_tree::ptree const &params)
{
}

void PopulateExifDataStage::Configure()
{
	LOG(2, "Configuring PopulateExifDataStage...");
	stream_ = app_->GetMainStream();
	info_ = app_->GetStreamInfo(stream_);
	LOG(2, "PopulateExifDataStage configured");
}


bool PopulateExifDataStage::Process(CompletedRequestPtr &completed_request)
{
	if (!stream_)
		return false;

	// auto exposure_time = completed_request->metadata.get(libcamera::controls::ExposureTime);
	// if (exposure_time)
	// 	completed_request->post_process_metadata.Set("exif_data.shutter_speed", (float)*exposure_time);

	// auto analogue_gain = completed_request->metadata.get(libcamera::controls::AnalogueGain);
	// if (analogue_gain)
	// 	completed_request->post_process_metadata.Set("exif_data.analogue_gain", *analogue_gain);

	// auto digital_gain = completed_request->metadata.get(libcamera::controls::DigitalGain);
	// if (digital_gain)
	// 	completed_request->post_process_metadata.Set("exif_data.digital_gain", *digital_gain);

	// auto lux = completed_request->metadata.get(libcamera::controls::Lux);
	// if (lux)
	// 	completed_request->post_process_metadata.Set("exif_data.frame_lux", *lux);

	return false;
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new PopulateExifDataStage(app);
}

static RegisterStage reg(NAME, &Create);