#include "QuadTree.h"
#include <queue>
#include <unordered_map>

#include "Pattern.h"

static constexpr int k_orb_half_boundary = 32;

// Constructor
QuadTree::QuadTree(cv::Mat image, const std::vector<cv::KeyPoint>& original_keypoints, int max_keypoint_count)
	: m_image(image)
{
	createTree(k_orb_half_boundary
		, k_orb_half_boundary
		, image.cols - k_orb_half_boundary
		, image.rows - k_orb_half_boundary
		, image
		, original_keypoints
		, max_keypoint_count
		);
}

void QuadTree::visualize() const
{
	// Used to show the output image.
	cv::Mat draw_image;
	cv::cvtColor(m_image, draw_image, cv::COLOR_GRAY2BGR);

	// BFS
	std::queue<size_t> que;
	que.push(0);
	while (!que.empty())
	{
		size_t idx = que.front();
		que.pop();

		cv::rectangle(draw_image,
			{ m_quad_tree[idx].x_min, m_quad_tree[idx].y_min },
			{ m_quad_tree[idx].x_max, m_quad_tree[idx].y_max },
			{ 0, 0, 255 },
			1
		);

		for (size_t i = 0; i < 4; ++i)
		{
			if (m_quad_tree[idx].next[i] != k_size_t_max)
			{
				que.push(m_quad_tree[idx].next[i]);
			}
		}
	}
	
	std::vector<cv::KeyPoint> keypoints;
	for (auto feature : m_features)
	{
		keypoints.push_back(feature->getKeyPoint());
	}

	cv::drawKeypoints(draw_image, keypoints, draw_image);

	cv::imshow("divided_image", draw_image);
	cv::waitKey(0);
	cv::destroyAllWindows();

	//cv::imwrite("origin_image.png", m_image);
	//cv::imwrite("optimized_image.png", draw_image);
}

std::vector<size_t> QuadTree::getKeyPointIndexInArea(float pos_x, float pos_y, float distance) const
{
	// Contain the indices of the keypoints need to return.
	std::vector<size_t> res;

	// Target point.
	const std::pair<float, float> point{ pos_x, pos_y };

	std::queue<size_t> que;
	que.push(0);
	// BFS
	while (!que.empty())
	{
		const auto& node = m_quad_tree[que.front()];
		que.pop();

		if (node.keypoint_idx != k_size_t_max)
		{
			// If the node is a leaf node and the distance is valid, then push the point into the res list.
			const auto& pt = m_features[node.keypoint_idx]->getKeyPoint().pt;
			if (QuadNode::calculateDistance(point, { pt.x, pt.y }) <= distance)
			{
				res.push_back(node.keypoint_idx);
			}
		}
		else
		{
			// If the node is a branch node, the check each next node if it should be push into the bfs queue.
			for (size_t next_idx : node.next)
			{
				if (next_idx == k_size_t_max) continue;

				float min_distance = m_quad_tree[next_idx].getMinDistanceToPoint(point);

				// If the point is contained by the node or the smallest distance from one corner to the point is less than distance,
				// then put the node into the queue.
				if (m_quad_tree[next_idx].containsPoint(point) || (min_distance <= distance))
				{
					que.push(next_idx);
				}
			}
		}
	}

	return res;
}

void QuadTree::createTree(int x_min, int y_min, int x_max, int y_max
	, cv::Mat image
	, const std::vector<cv::KeyPoint>& original_keypoints
	, int max_keypoint_count
	)
{
	// The original node contains the full image.
	m_quad_tree.emplace_back(x_min, y_min, x_max, y_max);
	std::queue<size_t> que;
	que.emplace(0);

	// Hash map for easy query.
	std::unordered_map<size_t, std::vector<size_t>> ump_keypoint_indices;
	{
		const size_t n = original_keypoints.size();
		std::vector<size_t> indices(n, 0);
		for (size_t i = 0; i < n; ++i)
		{
			indices[i] = i;
		}

		ump_keypoint_indices[0] = std::move(indices);
	}

	size_t end_node_count = 0;
	while ((!que.empty()) && (end_node_count < max_keypoint_count))
	{
		// Get the index of the node need to be divided.
		size_t node_idx = que.front();
		// Get the keypoint indices in the node.
		const auto& keypoint_indices = ump_keypoint_indices[node_idx];

		// Equaly divide the square into 4 pieces.
		const int x_mid = ((m_quad_tree[node_idx].x_min + m_quad_tree[node_idx].x_max) >> 1);
		const int y_mid = ((m_quad_tree[node_idx].y_min + m_quad_tree[node_idx].y_max) >> 1);

		// next_indices[i] will store the keypoint indices inside its square.
		std::array<std::vector<size_t>, 4> next_indices;
		// Square area bounds.
		const std::array<std::array<int, 4>, 4> bounds =
		{
			std::array<int, 4>{ m_quad_tree[node_idx].x_min, m_quad_tree[node_idx].y_min, x_mid, y_mid },
			std::array<int, 4>{ x_mid, m_quad_tree[node_idx].y_min, m_quad_tree[node_idx].x_max, y_mid },
			std::array<int, 4>{ m_quad_tree[node_idx].x_min, y_mid, x_mid, m_quad_tree[node_idx].y_max },
			std::array<int, 4>{ x_mid, y_mid, m_quad_tree[node_idx].x_max, m_quad_tree[node_idx].y_max },
		};

		// Distribute the keypoints.
		for (size_t idx : keypoint_indices)
		{
			const auto& point = original_keypoints[idx].pt;
			auto [x_pos, y_pos] = point;

			if (x_pos < x_min || x_pos >= x_max || y_pos < y_min || y_pos >= y_max) continue;

			const size_t block_idx = (static_cast<size_t>((x_pos <= x_mid) ? 0 : 1)) + ((y_pos <= y_mid) ? 0 : 2);
			next_indices[block_idx].push_back(idx);
		}

		bool has_next = false;
		for (size_t i = 0; i < 4; ++i)
		{
			if (!next_indices[i].empty())
			{
				has_next = true;

				m_quad_tree[node_idx].next[i] = m_quad_tree.size();

				// Create a new quad node if the node has not less than 1 keypoint.
				m_quad_tree.emplace_back(bounds[i][0], bounds[i][1], bounds[i][2], bounds[i][3]);
				++end_node_count;

				if (next_indices[i].size() > 1)
				{
					// Divide the node if it has more than 1 keypoint.
					que.push(m_quad_tree.size() - 1);
				}

				// The keypoint information will be store by each subnode.
				ump_keypoint_indices[m_quad_tree.size() - 1] = std::move(next_indices[i]);
			}
		}

		if (has_next)
		{
			--end_node_count;
		}

		// The parent node will no longer store the indices information.
		ump_keypoint_indices.erase(ump_keypoint_indices.find(node_idx));

		que.pop();
	}

	std::queue<size_t> bfs_que;
	bfs_que.push(0);

	const auto getBestKeypoint = [&](const std::vector<size_t>& range)->cv::KeyPoint
	{
		size_t idx = 0;
		float max_response = original_keypoints[range[0]].response;

		const size_t n = range.size();
		for (size_t i = 1; i < n; ++i)
		{
			if (max_response < original_keypoints[range[i]].response)
			{
				max_response = original_keypoints[range[i]].response;
				idx = i;
			}
		}

		return original_keypoints[range[idx]];
	};

	size_t leaf_count = 0;
	while (!bfs_que.empty())
	{
		size_t idx = bfs_que.front();
		bfs_que.pop();

		if (!m_quad_tree[idx].hasChild())
		{
			m_quad_tree[idx].keypoint_idx = m_features.size();
			auto kp = getBestKeypoint(ump_keypoint_indices[idx]);


			Feature::Descriptor descriptor;
			float m01 = 0;
			float m10 = 0;
			for (int dx = -k_orb_half_patch_size; dx < k_orb_half_patch_size; ++dx)
			{
				for (int dy = -k_orb_half_patch_size; dy < k_orb_half_patch_size; ++dy)
				{
					uchar pixel = image.at<uchar>(int(kp.pt.y) + dy, int(kp.pt.x) + dx);
					m10 += dx * pixel;
					m01 += dy * pixel;
				}
			}

			float sqrt_m = std::sqrt(m01 * m01 + m10 * m10) + 1e-5f;
			float sin_theta = m01 / sqrt_m;
			float cos_theta = m10 / sqrt_m;


			for (int i = 0; i < 256; ++i)
			{
				// default position
				cv::Point2f p((float)k_bit_pattern_31[i * 4], (float)k_bit_pattern_31[i * 4 + 1]);
				cv::Point2f q((float)k_bit_pattern_31[i * 4 + 2], (float)k_bit_pattern_31[i * 4 + 3]);

				// rotated position with theta
				cv::Point2f pp = cv::Point2f(cos_theta * p.x - sin_theta * p.y, sin_theta * p.x + cos_theta * p.y) + kp.pt;
				cv::Point2f qq = cv::Point2f(cos_theta * q.x - sin_theta * q.y, sin_theta * q.x + cos_theta * q.y) + kp.pt;

				descriptor[i] = (image.at<uchar>((int)pp.y, (int)pp.x) < image.at<uchar>((int)qq.y, (int)qq.x));
			}


			m_features.push_back(Feature::create(std::move(kp), std::move(descriptor)));

			++leaf_count;
		}
		else
		{
			for (size_t n : m_quad_tree[idx].next)
			{
				if (n != k_size_t_max)
				{
					bfs_que.push(n);
				}
			}
		}
	}
}
