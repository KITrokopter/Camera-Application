//#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <iostream>
#include <libfreenect.hpp>

#include "../src/CvKinect.hpp"
#include "../src/IImageReceiver.hpp"
#include "../src/ImageAnalyzer.hpp"
#include "../src/CvImageProcessor.hpp"

class TestCalibration : public IImageReceiver
{
private:
	cv::Mat* lastImage;
	
public:
	TestCalibration();
	void receiveImage(cv::Mat* image);
	~TestCalibration();
};

TestCalibration::TestCalibration()
{
	cv::startWindowThread();
	cv::namedWindow("Calibration Test");
	lastImage = 0;
}

void TestCalibration::receiveImage(cv::Mat* image) {
	cv::imshow("Calibration Test", *image);
	
	if (lastImage != 0) {
		delete lastImage;
	}
	
	lastImage = image;
}

TestCalibration::~TestCalibration() {
	if (lastImage != 0) {
		delete lastImage;
	}
	
	cv::destroyWindow("Calibration Test");
	
	std::cout << "TestCalibration deleted" << std::endl;
}

int main(int argc, char** argv)
{
	std::cout << "Starting ROS Node" << std::endl;
	ros::init(argc, argv, "TestTakePicture");
	ROS_DEBUG("Starting test");
	std::cout << "ROS Node started" << std::endl;
	
	Freenect::Freenect freenect;
	CvKinect& device = freenect.createDevice<CvKinect>(0);
	ImageAnalyzer analyzer(&device);
	TestCalibration tc;
	
	std::cout << "Starting calibration" << std::endl;
	
	CvImageProcessor processor(&analyzer);
	processor.startCalibration(3, 1000, 11, 8, 3, 3, &tc);
	processor.waitForCalibration();
	
	std::cout << "Shutting down ROS Node" << std::endl;
	ros::shutdown();
	std::cout << "ROS shutdown request sent" << std::endl;
}