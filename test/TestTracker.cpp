#include "../src/Tracker.hpp"
#include "../src/ITrackerDataReceiver.hpp"
#include "../src/CvKinect.hpp"
#include "../src/IImageReceiver.hpp"
#include "../src/ImageAnalyzer.hpp"
#include "../src/CvImageProcessor.hpp"

#include <iostream>
#include <opencv2/core/core.hpp>
#include <libfreenect.hpp>
#include <ros/ros.h>

#include <ctime>    // For time()
#include <cstdlib>  // For srand() and rand()

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
	ros::init(argc, argv, "TestTracker");
	
	/* if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) { */
	/* 	ros::console::notifyLoggerLevelsChanged(); */
	/* } */
	
	srand(time(0));
	
	QuadcopterColor* color = new QuadcopterColor(245, 10, 120, 255, 80, 255, 0);
	Tracker tracker(new TestTracker(), color);
	
	tracker.start();
	std::cout << "Tracker started" << std::endl;
	
	Freenect::Freenect freenect;
	CvKinect& device = freenect.createDevice<CvKinect>(0);
	CvImageProcessor processor(&device, 0);
	
	processor.start();
	usleep(1000 * 100);
	cv::Mat* mat = processor.getImage();
	processor.stop();
	
	/*cv::Mat* mat = new cv::Mat(cv::Size(640, 480), CV_8UC3);
	for (int i = 0; i < 640 * 480 * 3; i++) {
		mat->data[i] = rand() % 256;
	}*/
	
	tracker.setNextImage(mat, 5634);
	
	// Give tracker time to detect image before stopping.
	usleep(1000000);
	
	tracker.stop();
	std::cout << "Tracker stopped" << std::endl;
	
	tracker.join();
	std::cout << "Tracker joined" << std::endl;
	
	std::cout << "Shutting down ROS Node" << std::endl;
	ros::shutdown();
	std::cout << "ROS shutdown request sent" << std::endl;
	
	while (ros::ok()) {
		std::cout << "Waiting for ros to terminate" << std::endl;
		usleep(10000);
	}
}
