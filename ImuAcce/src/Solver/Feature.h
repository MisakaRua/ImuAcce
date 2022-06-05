#pragma once
#include <bitset>
#include <memory>
#include <unordered_map>
#include <type_traits>

#include <opencv2/opencv.hpp>

class MapPoint;

class Feature
{
public:
	using Ptr = std::shared_ptr<Feature>;
	using Library = std::unordered_map<size_t, Ptr>;
	static constexpr size_t k_descriptor_size = 256;
	using Descriptor = std::bitset<k_descriptor_size>;

private:
	Feature(size_t id, cv::KeyPoint kp, Descriptor descriptor)
		: m_id(id)
		, m_keypoint(std::move(kp))
		, m_descriptor(std::move(descriptor))
	{}
	Feature(const Feature&) = delete;
	Feature& operator=(const Feature&) = delete;

public:
	~Feature() noexcept = default;

	const auto& getKeyPoint() const { return m_keypoint; }
	const auto& getDescriptor() const { return m_descriptor; }

	void setMapPoint(std::shared_ptr<MapPoint> map_point) { m_map_point = map_point; }
	std::shared_ptr<MapPoint> getMapPoint() const { return m_map_point; }

	static void init()
	{
		s_features_library = new Library();
	}

	static void exit()
	{
		delete s_features_library;
	}

	template <typename... Args>
	static Ptr create(Args&&... args)
	{
		(*s_features_library)[s_feature_count] = Ptr(new Feature(s_feature_count, std::forward<Args>(args)...));
		return (*s_features_library)[s_feature_count++];
	}

	static void erase(Ptr feature)
	{
		auto it = s_features_library->find(feature->m_id);
		if (it != s_features_library->end())
		{
			s_features_library->erase(it);
		}
	}

private:
	inline static size_t s_feature_count = 0;
	inline static Library* s_features_library;

	size_t m_id = 0;
	cv::KeyPoint m_keypoint;
	Descriptor m_descriptor;

	std::shared_ptr<MapPoint> m_map_point;
};