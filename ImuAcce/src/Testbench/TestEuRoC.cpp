#include "TestEuRoC.h"

#include <opencv2/opencv.hpp>

TestEuRoC::TestEuRoC(const std::string& folder_path)
	: m_data_loader(folder_path)
{
}

void TestEuRoC::process_Iner() const
{
}
