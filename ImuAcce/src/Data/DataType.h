#pragma once
#include <vector>
#include <utility>

#include <opencv2/opencv.hpp>

struct ImgData
{
	size_t time_stamp = 0;
	cv::Mat image;
};

struct ImuData
{
	size_t time_stamp = 0;
	double wx = 0.0f;
	double wy = 0.0f;
	double wz = 0.0f;
	double ax = 0.0f;
	double ay = 0.0f;
	double az = 0.0f;
};

using ImgList = std::vector<ImgData>;
using ImuList = std::vector<ImuData>;
using DataListPair = std::pair<ImgList, ImuList>;