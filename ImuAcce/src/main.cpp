#include <vector>
#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

#include "Solver/Feature.h"
#include "Solver/KeyFrame.h"
#include "Solver/MapPoint.h"
#include "Testbench/TestQuadTree.h"
#include "Testbench/TestEuRoC.h"

static void testMain(int argc, char** argv)
{
	if (argc < 2)
	{
		std::cout << "Usage: ./ImuAcce [Test Type] [Test Parameters...]" << std::endl;
		return;
	}

	const std::string test_type(argv[1]);

	if (test_type == "QuadTree")
	{
		if (argc < 3)
		{
			std::cout << "Usage: ./ImuAcce QuadTree [Image Path]" << std::endl;
			return;
		}
		const std::string image_path(argv[2]);
		TestQuadTree tqt(image_path);
		tqt.process();
	}
	else if (test_type == "EuRoC")
	{
		TestEuRoC te("");
		te.process();
	}
}

int main(int argc, char** argv)
{
	Feature::init();
	KeyFrame::init();
	MapPoint::init();

	testMain(argc, argv);

	Feature::exit();
	KeyFrame::exit();
	MapPoint::exit();
}