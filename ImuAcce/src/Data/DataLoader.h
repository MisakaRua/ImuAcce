#pragma once
#include <optional>

#include <opencv2/opencv.hpp>

#include "DataType.h"

template <typename T>
class DataLoader
{
public:
	std::optional<DataListPair> getNextData() const
	{
		return static_cast<const T*>(this)->getNextData_Iner();
	}

	bool hasNext() const
	{
		return static_cast<const T*>(this)->hasNext_Iner();
	}
};