#pragma once

#include "TestBase.h"
#include "Solver/KeyFrame.h"

class TestQuadTree : public TestBase<TestQuadTree>
{
	friend class TestBase<TestQuadTree>;

public:
	TestQuadTree(const std::string& path)
		: m_keyframe(KeyFrame::create(getSuitableSizeImage(path)))
	{
		m_keyframe->visualize();
	}
	TestQuadTree(const TestQuadTree&) = delete;
	TestQuadTree& operator=(const TestQuadTree&) = delete;

private:
	void process_Iner() const
	{
		std::string win_name("ImuMonoAcce");

		cv::namedWindow(win_name);
		cv::Mat frame;
		m_keyframe->getQuadTree().m_image.copyTo(frame);


		struct UserData
		{
			cv::Point mouse_pos;
			cv::Mat image;
		};

		UserData user_data{};
		user_data.image = frame;


		cv::setMouseCallback(win_name, [](int event, int x, int y, int flags, void* userdata)
		{
			auto& [mouse_pos, image] = *static_cast<UserData*>(userdata);

			switch (event)
			{
			case cv::EVENT_MOUSEMOVE:
			{
				mouse_pos.x = std::clamp<int>(x, 0, image.cols - 1);
				mouse_pos.y = std::clamp<int>(y, 0, image.rows - 1);
				break;
			}
			default:
				break;
			}
		}, &user_data);


		size_t idx = 0;
		while (true)
		{
			std::cout
				<< "Frame Count (" << idx++ << ")\t"
				<< "Mouse Pos (" << user_data.mouse_pos.x << ", " << user_data.mouse_pos.y << ")" << std::endl;

			auto indices = m_keyframe->getQuadTree().getKeyPointIndexInArea(user_data.mouse_pos.x, user_data.mouse_pos.y, 100.0f);
			std::vector<cv::KeyPoint> kps;
			for (size_t idx : indices)
			{
				kps.push_back(m_keyframe->getQuadTree().m_features[idx]->getKeyPoint());
			}

			cv::Mat draw_image;
			cv::drawKeypoints(m_keyframe->getQuadTree().m_image, kps, draw_image);

			cv::circle(draw_image, cv::Point(user_data.mouse_pos.x, user_data.mouse_pos.y), 100, cv::Scalar(255, 255, 255));

			draw_image.copyTo(frame);
			cv::imshow(win_name, frame);

			//std::string name = "vedio\\" + std::to_string(idx++) + ".png";
			//cv::imwrite(name, draw_image);

			if (cv::waitKey(20) == 27) break;
		}
	}

private:
	static cv::Mat getSuitableSizeImage(const std::string& path)
	{
		cv::Mat image = cv::imread(path, cv::IMREAD_GRAYSCALE);
		static constexpr int max_size = 1080;
		const int ori_width = image.cols;
		const int ori_height = image.rows;
		int new_width;
		int new_height;
		if (ori_width > ori_height)
		{
			new_width = max_size;
			new_height = (int)(max_size * (float)ori_height / (float)ori_width);
		}
		else
		{
			new_height = max_size;
			new_width = (int)(max_size * (float)ori_width / (float)ori_height);
		}
		cv::resize(image, image, cv::Size(new_width, new_height));

		return image;
	}

private:
	KeyFrame::Ptr m_keyframe;
};