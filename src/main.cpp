#include <opencv2/core/mat.hpp>
#include <iostream>

#include "libfreenect.hpp"
#include "CvKinect.hpp"
#include "ImageAnalyzer.hpp"

int main(void) {
	std::cout << "Starting Camera Application" << std::endl;
	
	Freenect::Freenect freenect;
	CvKinect& device = freenect.createDevice<CvKinect>(0);
	ImageAnalyzer analyzer(&device);
	
	// Do ROS stuff
}
