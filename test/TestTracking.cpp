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
	void receiveImage(cv::Mat* image, long int time);
	~TestCalibration();
};

TestCalibration::TestCalibration()
{
	cv::startWindowThread();
	cv::namedWindow("Calibration Test");
	lastImage = 0;
}

void TestCalibration::receiveImage(cv::Mat* image, long int time) {
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
}

class TestTracker : public ITrackerDataReceiver
{
public:
	TestTracker(){}
	~TestTracker(){}
		
	void receiveTrackingData(cv::Scalar direction, int id, long int time)
	{
		std::cout << "Id: " << id << std::endl;
		std::cout << "Direction: " << direction.val[0] << ", " << direction.val[1] << ", " << direction.val[2] << std::endl;
	}
};

int main(int argc, char** argv)
{
	std::cout << "Starting ROS Node" << std::endl;
	ros::init(argc, argv, "TestTakePicture");
	
	/*if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		ros::console::notifyLoggerLevelsChanged();
	}*/
	
	ROS_DEBUG("Starting test");
	std::cout << "ROS Node started" << std::endl;
	
	Freenect::Freenect freenect;
	CvKinect& device = freenect.createDevice<CvKinect>(0);
	TestTracker tt;
	CvImageProcessor processor(&device, &tt);
	TestCalibration tc;
	
	std::cout << "Starting calibration" << std::endl;
	
	processor.addQuadcopter(new QuadcopterColor(245, 10, 120, 255, 80, 255, 0));
	
	processor.startCalibration(15, 500, 11, 8, 3, 3, &tc);
	processor.waitForCalibration();
	
	processor.start();
	processor.startTracking();
	
	usleep(1000 * 1000 * 10);
	
	processor.stopTracking();
	processor.stop();
	
	std::cout << "Shutting down ROS Node" << std::endl;
	ros::shutdown();
	std::cout << "ROS shutdown request sent" << std::endl;
	
	while (ros::ok()) {
		std::cout << "Waiting for ros to terminate" << std::endl;
		usleep(10000);
	}
	
	exit(0);
}
