#include "KeyFrame.h"

#include "Pattern.h"
#include "Feature.h"

KeyFrame::KeyFrame(size_t id, cv::Mat image)
	: m_id(id)
{
	std::vector<cv::KeyPoint> origin_keypoints;
	cv::FAST(image, origin_keypoints, 20);

	m_quad_tree = std::make_unique<QuadTree>(image, origin_keypoints);
}
