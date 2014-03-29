#include <opencv2/core/core.hpp>
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
	void receiveImage(cv::Mat* image, long int time, int type);
	~TestCalibration();
};

TestCalibration::TestCalibration()
{
	cv::startWindowThread();
	cv::namedWindow("Calibration Test");
	lastImage = 0;
}

void TestCalibration::receiveImage(cv::Mat* image, long int time, int type) {
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
	
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		ros::console::notifyLoggerLevelsChanged();
	}
	
	ROS_DEBUG("Starting test");
	std::cout << "ROS Node started" << std::endl;
	
	Freenect::Freenect freenect;
	CvKinect& device = freenect.createDevice<CvKinect>(0);
	TestTracker tt;
	CvImageProcessor processor(&device, &tt, true, true);
	TestCalibration tc;
	
	std::cout << "Starting calibration" << std::endl;
	
	processor.addQuadcopter(new QuadcopterColor(40, 78, 120, 255, 100, 255, 0)); // Green
//	processor.addQuadcopter(new QuadcopterColor(21, 37, 120, 255, 100, 255, 0)); // Yellow
	processor.addQuadcopter(new QuadcopterColor(163, 177, 120, 255, 100, 255, 0)); // Pink
//	processor.addQuadcopter(new QuadcopterColor(178, 13, 120, 255, 100, 255, 0)); // Orange
	
	processor.startCalibration(20, 500, 7, 7, 3, 3, &tc, 0);
	processor.waitForCalibration();
	
	processor.start();
	processor.startTracking();
	
	usleep(1000 * 1000 * 100);
	
	ROS_DEBUG("Stopping tracking");
	processor.stopTracking();
	processor.stop();
	ROS_DEBUG("Tracking stopped");
	
	std::cout << "Shutting down ROS Node" << std::endl;
	ros::shutdown();
	std::cout << "ROS shutdown request sent" << std::endl;
	
	while (ros::ok()) {
		std::cout << "Waiting for ros to terminate" << std::endl;
		usleep(10000);
	}
	
	exit(0);
}
