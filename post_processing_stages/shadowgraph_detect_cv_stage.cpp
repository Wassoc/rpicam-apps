/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * shadowgraph_detect_cv_stage.cpp - telecentric shadowgraph detector.
 *
 * Port of sg-object-detect/sg_object_detect.py. The base path ignores the
 * outer aperture band. Edge zone, annotation units, ROI crops, minimum size,
 * and the size distribution are off unless enabled. Boxes are drawn on the
 * frame that rpicam-raw encodes. Measurements are only collected as a JSON
 * sidecar when "json" is enabled, which needs a file output to receive it.
 */

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <libcamera/formats.h>

#include <nlohmann/json.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "core/logging.hpp"
#include "core/options.hpp"
#include "core/rpicam_app.hpp"
#include "core/shadowgraph_results.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

using json = nlohmann::json;

namespace
{

struct Found
{
	int id;
	int x;
	int y;
	int width;
	int height;
	int length_px;
	int thickness_px;
	int area;
	float angle_deg;
	cv::Point2f corners[4];
};

double roundTo(double value, int digits)
{
	double scale = std::pow(10.0, digits);
	return std::round(value * scale) / scale;
}

double percentile(const std::vector<double> &sorted_values, double percent)
{
	if (sorted_values.size() == 1)
		return sorted_values.front();
	double rank = (sorted_values.size() - 1) * (percent / 100.0);
	size_t low = static_cast<size_t>(rank);
	size_t high = std::min(low + 1, sorted_values.size() - 1);
	double weight = rank - static_cast<double>(low);
	return sorted_values[low] * (1.0 - weight) + sorted_values[high] * weight;
}

json summarize(const std::vector<double> &values, int digits)
{
	if (values.empty())
		return json();
	std::vector<double> ordered = values;
	std::sort(ordered.begin(), ordered.end());
	double mean = std::accumulate(ordered.begin(), ordered.end(), 0.0) / static_cast<double>(ordered.size());
	double sum_sq = 0.0;
	for (double value : ordered)
	{
		double delta = value - mean;
		sum_sq += delta * delta;
	}
	double spread = ordered.size() > 1 ? std::sqrt(sum_sq / static_cast<double>(ordered.size() - 1)) : 0.0;
	return json{
		{ "min", roundTo(ordered.front(), digits) },
		{ "p25", roundTo(percentile(ordered, 25), digits) },
		{ "median", roundTo(percentile(ordered, 50), digits) },
		{ "p75", roundTo(percentile(ordered, 75), digits) },
		{ "max", roundTo(ordered.back(), digits) },
		{ "mean", roundTo(mean, digits) },
		{ "stdev", roundTo(spread, digits) },
	};
}

std::string lowerCopy(std::string text)
{
	for (char &c : text)
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	return text;
}

bool isMono8(const libcamera::PixelFormat &fmt)
{
	return fmt == libcamera::formats::R8 || fmt == libcamera::formats::SBGGR8 ||
		   fmt == libcamera::formats::SGBRG8 || fmt == libcamera::formats::SGRBG8 ||
		   fmt == libcamera::formats::SRGGB8;
}

} // namespace

class ShadowgraphDetectCvStage : public PostProcessingStage
{
public:
	ShadowgraphDetectCvStage(RPiCamApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;
	void Read(boost::property_tree::ptree const &params) override;
	void Configure() override;
	bool Process(CompletedRequestPtr &completed_request) override;

private:
	cv::Mat shadowMask(const cv::Mat &img) const;
	bool allMeasuresBelow(int width_px, int height_px, int length_px, int thickness_px) const;
	double realIn(double pixels) const { return roundTo(pixels * in_per_px_, 4); }
	double realMm(double pixels) const { return roundTo(pixels * mm_per_px_, 3); }
	std::string annotationLabel(int object_id, int length_px, int thickness_px) const;
	json sizeDistribution(const std::vector<Found> &found) const;
	json detectionJson(const Found &found) const;

	libcamera::Stream *stream_ = nullptr;
	bool logged_format_ = false;

	double background_sigma_ = 35.0;
	double contrast_threshold_ = 12.0;
	int line_kernel_ = 11;
	double line_threshold_ = 10.0;
	double field_threshold_ = 50.0;
	double field_margin_ = 90.0;
	double edge_inner_ = 4.0;
	int min_area_ = 20;
	int max_area_ = 100000;
	double imaging_diameter_px_ = 1220.0;
	double imaging_diameter_in_ = 1.65;
	double imaging_diameter_mm_ = 42.0;
	double in_per_px_ = 1.65 / 1220.0;
	double mm_per_px_ = 42.0 / 1220.0;

	bool edge_zone_ = false;
	std::string units_ = "mm";
	bool rois_ = false;
	bool size_distribution_ = false;
	bool json_ = false;
	bool min_size_enabled_ = false;
	double min_size_ = 0.0;
	std::string min_size_unit_ = "mm";
	int line_thickness_ = 2;
	double font_size_ = 0.45;
};

#define NAME "shadowgraph_detect_cv"

char const *ShadowgraphDetectCvStage::Name() const
{
	return NAME;
}

void ShadowgraphDetectCvStage::Read(boost::property_tree::ptree const &params)
{
	background_sigma_ = params.get<double>("background_sigma", background_sigma_);
	contrast_threshold_ = params.get<double>("contrast_threshold", contrast_threshold_);
	line_kernel_ = params.get<int>("line_kernel", line_kernel_);
	line_threshold_ = params.get<double>("line_threshold", line_threshold_);
	field_threshold_ = params.get<double>("field_threshold", field_threshold_);
	field_margin_ = params.get<double>("field_margin", field_margin_);
	edge_inner_ = params.get<double>("edge_inner", edge_inner_);
	min_area_ = params.get<int>("min_area", min_area_);
	max_area_ = params.get<int>("max_area", max_area_);
	imaging_diameter_px_ = params.get<double>("imaging_diameter_px", imaging_diameter_px_);
	imaging_diameter_in_ = params.get<double>("imaging_diameter_in", imaging_diameter_in_);
	imaging_diameter_mm_ = params.get<double>("imaging_diameter_mm", imaging_diameter_mm_);
	edge_zone_ = params.get<bool>("edge_zone", edge_zone_);
	units_ = params.get<std::string>("units", units_);
	rois_ = params.get<bool>("rois", rois_);
	size_distribution_ = params.get<bool>("size_distribution", size_distribution_);
	json_ = params.get<bool>("json", json_);
	line_thickness_ = params.get<int>("line_thickness", line_thickness_);
	font_size_ = params.get<double>("font_size", font_size_);
	std::string min_size_text = params.get<std::string>("min_size", "");

	if (app_->GetOptions()->Get().shadowgraph_detect)
	{
		const auto &opt = app_->GetOptions()->Get();
		edge_zone_ = opt.shadowgraph_edge_zone;
		units_ = opt.shadowgraph_units;
		rois_ = opt.shadowgraph_rois;
		size_distribution_ = opt.shadowgraph_size_distribution;
		json_ = opt.shadowgraph_json;
		min_size_text = opt.shadowgraph_min_size;
	}

	if (imaging_diameter_px_ <= 0.0)
		throw std::runtime_error("shadowgraph_detect_cv: imaging_diameter_px must be positive");
	in_per_px_ = imaging_diameter_in_ / imaging_diameter_px_;
	mm_per_px_ = imaging_diameter_mm_ / imaging_diameter_px_;
	if (line_kernel_ < 1)
		line_kernel_ = 1;
	if (line_thickness_ < 1)
		line_thickness_ = 1;

	units_ = lowerCopy(units_);
	if (units_ != "mm" && units_ != "in" && units_ != "px")
		throw std::runtime_error("shadowgraph_detect_cv: units must be mm, in, or px");

	min_size_enabled_ = !min_size_text.empty();
	if (min_size_enabled_)
	{
		std::string error;
		if (!ParseShadowgraphMinSize(min_size_text, min_size_, min_size_unit_, error))
			throw std::runtime_error("shadowgraph_detect_cv: " + error);
	}
}

void ShadowgraphDetectCvStage::Configure()
{
	stream_ = app_->GetMainStream();
	if (!stream_)
		throw std::runtime_error("shadowgraph_detect_cv: no camera stream");
}

cv::Mat ShadowgraphDetectCvStage::shadowMask(const cv::Mat &img) const
{
	cv::Mat background;
	cv::GaussianBlur(img, background, cv::Size(0, 0), background_sigma_);

	cv::Mat bright;
	cv::compare(background, field_threshold_, bright, cv::CMP_GT);

	cv::Mat distance;
	cv::distanceTransform(bright, distance, cv::DIST_L2, 5);

	cv::Mat interior;
	cv::compare(distance, field_margin_, interior, cv::CMP_GE);

	cv::Mat difference, blobs;
	cv::absdiff(img, background, difference);
	cv::threshold(difference, blobs, contrast_threshold_, 255, cv::THRESH_BINARY);

	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(line_kernel_, line_kernel_));
	cv::Mat blackhat, lines;
	cv::morphologyEx(img, blackhat, cv::MORPH_BLACKHAT, kernel);
	cv::threshold(blackhat, lines, line_threshold_, 255, cv::THRESH_BINARY);

	cv::Mat foreground, mask;
	cv::bitwise_or(blobs, lines, foreground);
	cv::bitwise_and(foreground, interior, mask);

	if (edge_zone_)
	{
		// Black-hat only. The Gaussian difference treats the whole falloff as one object.
		cv::Mat ge_inner, lt_margin, band, edge_lines;
		cv::compare(distance, edge_inner_, ge_inner, cv::CMP_GE);
		cv::compare(distance, field_margin_, lt_margin, cv::CMP_LT);
		cv::bitwise_and(ge_inner, lt_margin, band);
		cv::bitwise_and(lines, band, edge_lines);
		cv::bitwise_or(mask, edge_lines, mask);
	}
	return mask;
}

bool ShadowgraphDetectCvStage::allMeasuresBelow(int width_px, int height_px, int length_px, int thickness_px) const
{
	double measures[4];
	if (min_size_unit_ == "px")
	{
		measures[0] = width_px;
		measures[1] = height_px;
		measures[2] = length_px;
		measures[3] = thickness_px;
	}
	else if (min_size_unit_ == "in")
	{
		measures[0] = realIn(width_px);
		measures[1] = realIn(height_px);
		measures[2] = realIn(length_px);
		measures[3] = realIn(thickness_px);
	}
	else
	{
		measures[0] = realMm(width_px);
		measures[1] = realMm(height_px);
		measures[2] = realMm(length_px);
		measures[3] = realMm(thickness_px);
	}
	return measures[0] < min_size_ && measures[1] < min_size_ && measures[2] < min_size_ && measures[3] < min_size_;
}

std::string ShadowgraphDetectCvStage::annotationLabel(int object_id, int length_px, int thickness_px) const
{
	std::ostringstream text;
	text << object_id << " ";
	if (units_ == "px")
		text << length_px << "x" << thickness_px << " px";
	else if (units_ == "in")
	{
		text.setf(std::ios::fixed);
		text.precision(4);
		text << realIn(length_px) << "x" << realIn(thickness_px) << " in";
	}
	else
	{
		text.setf(std::ios::fixed);
		text.precision(3);
		text << realMm(length_px) << "x" << realMm(thickness_px) << " mm";
	}
	return text.str();
}

json ShadowgraphDetectCvStage::detectionJson(const Found &found) const
{
	return json{
		{ "id", found.id },
		{ "x_px", found.x },
		{ "y_px", found.y },
		{ "width_px", found.width },
		{ "width_in", realIn(found.width) },
		{ "width_mm", realMm(found.width) },
		{ "height_px", found.height },
		{ "height_in", realIn(found.height) },
		{ "height_mm", realMm(found.height) },
		{ "length_px", found.length_px },
		{ "length_in", realIn(found.length_px) },
		{ "length_mm", realMm(found.length_px) },
		{ "thickness_px", found.thickness_px },
		{ "thickness_in", realIn(found.thickness_px) },
		{ "thickness_mm", realMm(found.thickness_px) },
		{ "angle_deg", roundTo(found.angle_deg, 1) },
		{ "area_px", found.area },
	};
}

json ShadowgraphDetectCvStage::sizeDistribution(const std::vector<Found> &found) const
{
	if (found.empty())
		return json{ { "count", 0 } };

	std::vector<double> length_px, length_mm, length_in, thickness_px, thickness_mm, thickness_in;
	std::vector<double> area_px, area_mm2, area_in2;
	length_px.reserve(found.size());
	for (const Found &item : found)
	{
		length_px.push_back(item.length_px);
		length_mm.push_back(realMm(item.length_px));
		length_in.push_back(realIn(item.length_px));
		thickness_px.push_back(item.thickness_px);
		thickness_mm.push_back(realMm(item.thickness_px));
		thickness_in.push_back(realIn(item.thickness_px));
		area_px.push_back(item.area);
		area_mm2.push_back(item.area * mm_per_px_ * mm_per_px_);
		area_in2.push_back(item.area * in_per_px_ * in_per_px_);
	}
	return json{
		{ "count", found.size() },
		{ "length_px", summarize(length_px, 2) },
		{ "length_mm", summarize(length_mm, 3) },
		{ "length_in", summarize(length_in, 4) },
		{ "thickness_px", summarize(thickness_px, 2) },
		{ "thickness_mm", summarize(thickness_mm, 3) },
		{ "thickness_in", summarize(thickness_in, 4) },
		{ "area_px", summarize(area_px, 1) },
		{ "area_mm2", summarize(area_mm2, 4) },
		{ "area_in2", summarize(area_in2, 6) },
	};
}

bool ShadowgraphDetectCvStage::Process(CompletedRequestPtr &completed_request)
{
	if (!stream_ || !completed_request->buffers.count(stream_))
		return false;

	StreamInfo info = app_->GetStreamInfo(stream_);
	const libcamera::PixelFormat &fmt = info.pixel_format;
	const bool mono8 = isMono8(fmt);
	const bool yuv = fmt == libcamera::formats::YUV420;
	const bool bgr = fmt == libcamera::formats::BGR888;
	const bool rgb = fmt == libcamera::formats::RGB888;
	if (!mono8 && !yuv && !bgr && !rgb)
	{
		if (!logged_format_)
		{
			logged_format_ = true;
			LOG_ERROR("shadowgraph_detect_cv: unsupported pixel format "
					  << fmt.toString()
					  << "; detection expects 8-bit mono (R8 or 8-bit Bayer), YUV420, BGR888, or RGB888");
		}
		return false;
	}
	if (info.stride < info.width || info.width == 0 || info.height == 0)
	{
		LOG_ERROR("shadowgraph_detect_cv: invalid frame geometry");
		return false;
	}

	BufferWriteSync writer(app_, completed_request->buffers[stream_]);
	libcamera::Span<uint8_t> buffer = writer.Get()[0];
	const int type = (bgr || rgb) ? CV_8UC3 : CV_8UC1;
	cv::Mat view(info.height, info.width, type, buffer.data(), info.stride);
	cv::Mat gray;
	cv::Scalar colour(255);
	if (bgr || rgb)
	{
		cv::cvtColor(view, gray, bgr ? cv::COLOR_BGR2GRAY : cv::COLOR_RGB2GRAY);
		colour = cv::Scalar(0, 255, 0);
	}
	else
		gray = view.clone();

	auto started = std::chrono::steady_clock::now();
	cv::Mat mask = shadowMask(gray);
	cv::Mat labels, stats, centroids;
	int count = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);

	std::vector<Found> found;
	std::vector<ShadowgraphRoi> roi_pngs;
	for (int i = 1; i < count; i++)
	{
		int x = stats.at<int>(i, cv::CC_STAT_LEFT);
		int y = stats.at<int>(i, cv::CC_STAT_TOP);
		int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
		int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
		int area = stats.at<int>(i, cv::CC_STAT_AREA);
		if (area < min_area_ || area > max_area_ || w <= 0 || h <= 0)
			continue;
		if (x < 0 || y < 0 || x + w > gray.cols || y + h > gray.rows)
			continue;

		cv::Mat component;
		cv::compare(labels(cv::Rect(x, y, w, h)), i, component, cv::CMP_EQ);
		std::vector<cv::Point> points;
		cv::findNonZero(component, points);
		if (points.empty())
			continue;

		cv::RotatedRect local = cv::minAreaRect(points);
		float side_w = local.size.width;
		float side_h = local.size.height;
		float angle = local.angle;
		float length, thickness;
		if (side_w < side_h)
		{
			length = side_h;
			thickness = side_w;
			angle += 90.f;
		}
		else
		{
			length = side_w;
			thickness = side_h;
		}
		if (angle <= -90.f)
			angle += 180.f;
		if (angle > 90.f)
			angle -= 180.f;

		int length_px = static_cast<int>(std::round(length));
		int thickness_px = static_cast<int>(std::round(thickness));
		if (min_size_enabled_ && allMeasuresBelow(w, h, length_px, thickness_px))
			continue;

		Found item;
		item.id = static_cast<int>(found.size()) + 1;
		item.x = x;
		item.y = y;
		item.width = w;
		item.height = h;
		item.length_px = length_px;
		item.thickness_px = thickness_px;
		item.area = area;
		item.angle_deg = angle;

		cv::RotatedRect rect(cv::Point2f(local.center.x + x, local.center.y + y), local.size, local.angle);
		rect.points(item.corners);

		if (rois_)
		{
			std::vector<uchar> encoded;
			cv::Mat crop = gray(cv::Rect(x, y, w, h)).clone();
			if (cv::imencode(".png", crop, encoded))
				roi_pngs.push_back({ item.id, std::vector<uint8_t>(encoded.begin(), encoded.end()) });
			else
				LOG_ERROR("shadowgraph_detect_cv: failed to encode ROI " << item.id);
		}
		found.push_back(item);
	}

	const int font = cv::FONT_HERSHEY_SIMPLEX;
	for (const Found &item : found)
	{
		std::vector<cv::Point> contour(4);
		int top = 0;
		for (int k = 0; k < 4; k++)
		{
			contour[k] = cv::Point(cvRound(item.corners[k].x), cvRound(item.corners[k].y));
			if (item.corners[k].y < item.corners[top].y)
				top = k;
		}
		std::vector<std::vector<cv::Point>> contours{ contour };
		cv::drawContours(view, contours, 0, colour, line_thickness_, cv::LINE_AA);
		int text_x = contour[top].x;
		int text_y = std::max(15, contour[top].y - 5);
		cv::putText(view, annotationLabel(item.id, item.length_px, item.thickness_px), cv::Point(text_x, text_y),
					font, font_size_, colour, 1, cv::LINE_AA);
	}

	// Nothing collects a sidecar unless a file is being written, so skip the work when both are off.
	if (json_ || !roi_pngs.empty())
	{
		ShadowgraphSidecar sidecar;
		if (json_)
		{
			json detections = json::array();
			for (const Found &item : found)
				detections.push_back(detectionJson(item));

			json document = {
				{ "image", "" },
				{ "image_width_px", info.width },
				{ "image_height_px", info.height },
				{ "imaging_diameter_px", imaging_diameter_px_ },
				{ "imaging_diameter_in", imaging_diameter_in_ },
				{ "imaging_diameter_mm", imaging_diameter_mm_ },
				{ "edge_zone", edge_zone_ },
				{ "annotation_units", units_ },
				{ "rois", rois_ },
				{ "min_size", min_size_enabled_ ? json(min_size_) : json() },
				{ "min_size_unit", min_size_enabled_ ? json(min_size_unit_) : json() },
				{ "size_distribution", size_distribution_ ? sizeDistribution(found) : json() },
				{ "object_count", found.size() },
				{ "detections", detections },
			};
			sidecar.json = document.dump(2);
		}
		sidecar.rois = std::move(roi_pngs);
		auto ts = completed_request->metadata.get(controls::FrameWallClock);
		int64_t timestamp_us = ts ? *ts : completed_request->buffers[stream_]->metadata().timestamp / 1000;
		ShadowgraphResults::Publish(timestamp_us, std::move(sidecar));
	}

	double elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
	LOG(1, "shadowgraph: " << found.size() << " shadows in " << static_cast<int>(elapsed_ms) << " ms");
	return false;
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new ShadowgraphDetectCvStage(app);
}

static RegisterStage reg(NAME, &Create);

// Referenced from rpicam_app so the linker keeps this translation unit.
void ShadowgraphDetectCvEnsureLinked() {}
