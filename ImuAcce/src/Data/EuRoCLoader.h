#pragma once
#include <vector>
#include <string>
#include <optional>

#include "DataType.h"
#include "DataLoader.h"

class EuRoCLoader : public DataLoader<EuRoCLoader>
{
	friend class DataLoader<EuRoCLoader>;

public:
	EuRoCLoader(const std::string& folder_path);
	EuRoCLoader(const EuRoCLoader&) = delete;
	EuRoCLoader& operator=(const EuRoCLoader&) = delete;

private:
	std::optional<DataListPair> getNextData_Iner() const;
	bool hasNext_Iner() const
	{
		return m_next_index < m_img_time_stamps.size();
	}

private:
	const std::string m_folder_path;
	std::vector<size_t> m_img_time_stamps;
	mutable std::vector<std::vector<ImuData>> m_imu_data;
	mutable size_t m_next_index = 0;
};