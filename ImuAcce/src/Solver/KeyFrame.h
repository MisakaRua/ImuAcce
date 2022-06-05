#pragma once
#include <memory>
#include <unordered_map>

#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>

#include "QuadTree.h"

class Feature;

class KeyFrame
{
public:
	using Ptr = std::shared_ptr<KeyFrame>;
	using Library = std::unordered_map<size_t, Ptr>;

private:
	KeyFrame(size_t id, cv::Mat image);
	KeyFrame(const KeyFrame&) = delete;
	KeyFrame& operator=(const KeyFrame&) = delete;

public:
	~KeyFrame() noexcept = default;

	QuadTree& getQuadTree() { return *m_quad_tree; }
	const QuadTree& getQuadTree() const { return *m_quad_tree; }

	void setPose(const Sophus::SE3f& new_pose) { m_pose = new_pose; }
	const auto& getPose() const { return m_pose; }

	void visualize() const { m_quad_tree->visualize(); }

	static void init()
	{
		s_keyframe_library = new Library();
	}

	static void exit()
	{
		delete s_keyframe_library;
	}
	
	template <typename... Args>
	static Ptr create(Args&&... args)
	{
		(*s_keyframe_library)[s_keyframe_count] = Ptr(new KeyFrame(s_keyframe_count, std::forward<Args>(args)...));
		return (*s_keyframe_library)[s_keyframe_count++];
	}

	static void erase(Ptr key_frame)
	{
		auto it = s_keyframe_library->find(key_frame->m_id);
		if (it != s_keyframe_library->end())
		{
			s_keyframe_library->erase(it);
		}
	}

private:
	inline static size_t s_keyframe_count = 0;
	inline static Library* s_keyframe_library;

	size_t m_id = 0;
	Sophus::SE3f m_pose;

	std::unique_ptr<QuadTree> m_quad_tree;
};