#pragma once
#include <utility>
#include <limits>
#include <vector>
#include <array>
#include <bitset>

#include <opencv2/opencv.hpp>

#include "Feature.h"

struct QuadTree
{
	static constexpr size_t k_size_t_max = std::numeric_limits<size_t>::max();
	static constexpr int k_int_max = std::numeric_limits<int>::max();

	struct QuadNode
	{
		std::array<size_t, 4> next =
		{
			k_size_t_max,
			k_size_t_max,
			k_size_t_max,
			k_size_t_max
		};
		int x_min = k_int_max;
		int y_min = k_int_max;
		int x_max = k_int_max;
		int y_max = k_int_max;

		size_t keypoint_idx = std::numeric_limits<size_t>::max();

		QuadNode() = default;
		QuadNode(int x_min, int y_min, int x_max, int y_max)
			: x_min(x_min)
			, y_min(y_min)
			, x_max(x_max)
			, y_max(y_max)
		{}

		bool hasChild() const
		{
			return (next[0] != k_size_t_max)
				|| (next[1] != k_size_t_max)
				|| (next[2] != k_size_t_max)
				|| (next[3] != k_size_t_max);
		}

		bool containsPoint(const std::pair<float, float>& p) const
		{
			return ((x_min <= p.first) && (p.first < x_max) && (y_min <= p.second) && (p.second < y_max));
		}

		float getMinDistanceToPoint(const std::pair<float, float>& p) const
		{
			const float d0 = calculateDistance(p, { x_min + 0.5f, y_min + 0.5f });
			const float d1 = calculateDistance(p, { x_min + 0.5f, y_max + 0.5f });
			const float d2 = calculateDistance(p, { x_max + 0.5f, y_min + 0.5f });
			const float d3 = calculateDistance(p, { x_max + 0.5f, y_max + 0.5f });

			return std::min(std::min(d0, d1), std::min(d2, d3));
		}

		static float calculateDistance(const std::pair<float, float>& p1, const std::pair<float, float>& p2)
		{
			const float dx = p1.first - p2.first;
			const float dy = p1.second - p2.second;
			return std::sqrt(dx * dx + dy * dy);
		};
	};

	cv::Mat m_image;
	std::vector<QuadNode> m_quad_tree;
	std::vector<Feature::Ptr> m_features;


	QuadTree(cv::Mat image, const std::vector<cv::KeyPoint>& original_keypoints, int max_keypoint_count = 1000);
	QuadTree(const QuadTree&) = delete;
	QuadTree& operator=(const QuadTree&) = delete;

	std::vector<size_t> getKeyPointIndexInArea(float pos_x, float pos_y, float distance) const;

	void visualize() const;

private:
	void createTree(int x_min, int y_min, int x_max, int y_max, cv::Mat image, const std::vector<cv::KeyPoint>& original_keypoints, int max_keypoint_count);
};