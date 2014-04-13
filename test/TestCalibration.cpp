#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <iostream>
#include <libfreenect.hpp>
#include <string>

#include "../src/CvKinect.hpp"
#include "../src/IImageReceiver.hpp"
#include "../src/ImageAnalyzer.hpp"
#include "../src/CvImageProcessor.hpp"

/**
 * Displays the video feed from the camera.
 *
 * @author Sebastian Schmidt
 */
class TestCalibration : public IImageReceiver {
private:
	cv::Mat *lastImage;
	std::string name;

public:
	TestCalibration(std::string name);
	void receiveImage(cv::Mat *image, long int time, int type);

	~TestCalibration();
};

TestCalibration::TestCalibration(std::string name)
{
	this->name = name;
	cv::startWindowThread();
	cv::namedWindow(name);
	lastImage = 0;
}

void TestCalibration::receiveImage(cv::Mat *image, long int time, int type)
{
	cv::imshow(name, *image);

	std::cout << "Image: " << image << " Type: " << type << std::endl;

	if (lastImage != 0)
		delete lastImage;

	lastImage = image;
}

TestCalibration::~TestCalibration()
{
	if (lastImage != 0)
		delete lastImage;

	cv::destroyWindow(name);
}

/**
 * Test to ensure calibration works.
 */
int main(int argc, char **argv)
{
	std::cout << "Starting ROS Node" << std::endl;
	ros::init(argc, argv, "TestTakePicture");
	ROS_DEBUG("Starting test");
	std::cout << "ROS Node started" << std::endl;

	Freenect::Freenect freenect;
	CvKinect &device = freenect.createDevice<CvKinect>(0);
	CvImageProcessor processor(&device, 0);
	TestCalibration tc("tc");

	std::cout << "Starting calibration" << std::endl;

	processor.startCalibration(3, 1000, 11, 8, 3, 3, &tc, 0);
	processor.waitForCalibration();

	std::cout << "Shutting down ROS Node" << std::endl;
	ros::shutdown();
	std::cout << "ROS shutdown request sent" << std::endl;

	while (ros::ok()) {
		std::cout << "Waiting for ros to terminate" << std::endl;
		usleep(10000);
	}

	exit(0);
}