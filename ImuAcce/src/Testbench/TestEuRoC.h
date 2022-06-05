#pragma once
#include <string>
#include <memory>

#include "Data/DataType.h"
#include "TestBase.h"
#include "Data/EuRoCLoader.h"

class TestEuRoC : public TestBase<TestEuRoC>
{
	friend class TestBase<TestEuRoC>;

public:
	TestEuRoC(const std::string& folder_path);

private:
	void process_Iner() const;

private:
	EuRoCLoader m_data_loader;
};