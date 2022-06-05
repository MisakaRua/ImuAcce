#include "EuRoCLoader.h"
#include <fstream>
#include <sstream>
#include <cassert>

EuRoCLoader::EuRoCLoader(const std::string& folder_path)
	: m_folder_path(folder_path + "\\mav0\\")
{
	{
		std::ifstream file(m_folder_path + "cam0\\data.csv");
		assert(file.is_open());
		std::string line;
		std::getline(file, line);

		while (std::getline(file, line))
		{
			std::string str;
			std::istringstream iss(line);
			std::getline(iss, str, ',');
			m_img_time_stamps.push_back(std::stoull(str));
		}
	}

	{
		std::ifstream file(m_folder_path + "imu0\\data.csv");
		assert(file.is_open());
		std::string line;
		std::getline(file, line);

		size_t img_idx = 0;
		std::vector<ImuData> imu_data;
		while (std::getline(file, line))
		{
			std::istringstream iss(line);
			std::string str;
			std::getline(iss, str, ',');
			size_t ts = std::stoull(str);

			if (img_idx < m_img_time_stamps.size() && ts > m_img_time_stamps[img_idx])
			{
				m_imu_data.push_back(std::move(imu_data));
				++img_idx;
			}

			ImuData id;
			id.time_stamp = ts;
			std::getline(iss, str, ','); id.wx = std::stod(str);
			std::getline(iss, str, ','); id.wy = std::stod(str);
			std::getline(iss, str, ','); id.wz = std::stod(str);
			std::getline(iss, str, ','); id.ax = std::stod(str);
			std::getline(iss, str, ','); id.ay = std::stod(str);
			std::getline(iss, str, ','); id.az = std::stod(str);
			imu_data.push_back(id);
		}
	}
}

auto EuRoCLoader::getNextData_Iner() const->std::optional<DataListPair>
{
	std::pair<std::vector<ImgData>, std::vector<ImuData>> res;
	
	std::ostringstream oss;
	oss << m_folder_path << "cam0\\data\\" << m_img_time_stamps[m_next_index] << ".png";
	cv::Mat img0 = cv::imread(oss.str(), cv::IMREAD_GRAYSCALE);
	assert(img0.data);
	res.first.push_back({ m_img_time_stamps[m_next_index], img0 });

	res.second = std::move(m_imu_data[m_next_index]);

	++m_next_index;

	return res;
}
