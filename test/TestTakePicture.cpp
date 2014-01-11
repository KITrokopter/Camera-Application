#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/core/mat.hpp>
#include <ros/console.h>
#include <ros/init.h>
#include <iostream>

#include "libfreenect.hpp"
#include "../src/CvKinect.hpp"
#include "../src/ImageAnalyzer.hpp"

int main(int argc, char** argv) {
	std::cout << "Starting ROS Node" << std::endl;
	ros::init(argc, argv, "TestTakePicture");
	ROS_DEBUG("Starting test");
	std::cout << "ROS Node started" << std::endl;
	
	Freenect::Freenect freenect;
	CvKinect& device = freenect.createDevice<CvKinect>(0);
	ImageAnalyzer analyzer(&device);
	analyzer.start();
	
	std::cout << "ImageAnalyzer started" << std::endl;
	
	cv::Mat* image = 0;
	
	do {
		image = analyzer.getImage();
	} while (image == 0);
	
	std::cout << "Got image" << std::endl;
	
	cv::namedWindow("rgb", CV_WINDOW_AUTOSIZE);
	cv::imshow("rgb", *image);
	cv::waitKey();
	cv::destroyWindow("rgb");
	
	analyzer.stop();
	delete image;
}
