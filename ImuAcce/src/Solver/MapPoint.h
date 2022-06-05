#pragma once
#include <memory>
#include <unordered_set>
#include <unordered_map>
#include <type_traits>

#include <Eigen/Core>

class Feature;

class MapPoint
{
public:
	using Ptr = std::shared_ptr<MapPoint>;
	using Library = std::unordered_map<size_t, Ptr>;
	using Pos = Eigen::Vector3f;

private:
	MapPoint(size_t id, const Pos& position)
		: m_id(id)
		, m_position(position)
	{}
	MapPoint(const MapPoint&) = delete;
	MapPoint& operator=(const MapPoint&) = delete;

public:
	~MapPoint() noexcept = default;

	void setPos(const Pos& new_pos) { m_position = new_pos; }
	const Pos& getPos() const { return m_position; }

	void addObservation(std::shared_ptr<Feature> feature)
	{
		m_observations.insert(feature);
	}
	void removeObservation(std::shared_ptr<Feature> feature)
	{
		if (m_observations.find(feature) != m_observations.end())
		{
			m_observations.erase(feature);
		}
	}
	const auto& getObservation() const { return m_observations; }

	static void init()
	{
		s_map_points_library = new Library();
	}

	static void exit()
	{
		delete s_map_points_library;
	}

	template <typename... Args>
	static Ptr create(Args&&... args)
	{
		(*s_map_points_library)[s_map_count] = Ptr(new MapPoint(s_map_count, std::forward<Args>(args)...));
		return (*s_map_points_library)[s_map_count++];
	}

	static void erase(Ptr map_point)
	{
		auto it = s_map_points_library->find(map_point->m_id);
		if (it != s_map_points_library->end())
		{
			s_map_points_library->erase(it);
		}
	}

private:
	inline static size_t s_map_count;
	inline static Library* s_map_points_library = nullptr;

	size_t m_id = 0;
	Pos m_position;
	std::unordered_set<std::shared_ptr<Feature>> m_observations;
};