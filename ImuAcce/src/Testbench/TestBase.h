#pragma once
#include <memory>

template <typename T>
class TestBase
{
public:
	void process() const
	{
		static_cast<const T*>(this)->process_Iner();
	}
};