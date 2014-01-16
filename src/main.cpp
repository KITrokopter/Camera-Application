//#include <opencv2/core/mat.hpp>
#include <iostream>

#include "ros/ros.h"

#include "libfreenect.hpp"
#include "CvKinect.hpp"
#include "ImageAnalyzer.hpp"
#include "Communicator.hpp"

int main(int argc, char **argv) {
	std::cout << "Starting Camera Application" << std::endl;
	
	/* Freenect::Freenect freenect; */
	/* CvKinect& device = freenect.createDevice<CvKinect>(0); */
	/* ImageAnalyzer analyzer(&device); */
	
	// Do ROS stuff
	ros::init(argc, argv, "camera_application");
	Communicator comm;
	ROS_INFO("Initialized.");
	ros::spin();
}
