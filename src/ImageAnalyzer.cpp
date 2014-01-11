#include "libfreenect.hpp"
#include "ImageAnalyzer.hpp"
#include <iostream>


ImageAnalyzer::ImageAnalyzer(CvKinect* imageSource) {
	this->imageSource = imageSource;
	imageSource->setImageReceiver(this);
	lastImage = 0;
}

void ImageAnalyzer::start() {
	imageSource->startVideo();
}

void ImageAnalyzer::stop() {
	imageSource->stopVideo();
}

// Returns a shallow copy of the last image. The image matrix is the same as the image matrix of the internal image.
cv::Mat* ImageAnalyzer::getImage() {
	if (lastImage == 0) {
		return 0;
	} else {
		std::cout << "IA has image" << std::endl;
		
		return new cv::Mat(*lastImage);
	}
}

void ImageAnalyzer::receiveImage(cv::Mat* image) {
	lastImage = image;
}

ImageAnalyzer::~ImageAnalyzer() {
	if (lastImage != 0) {
		delete lastImage;
	}
}

// int main(void) {	
// 	cv::namedWindow("rgb", CV_WINDOW_AUTOSIZE);
// 	cv::namedWindow("depth", CV_WINDOW_AUTOSIZE);
// 	
// 	while (true) {
// 		device.getVideo(rgbMat);
// 		
// 		// if (images.size() > 0) {
// 		// 	cv::imshow("rgb", images.front());
// 		// } else {
// 			cv::imshow("rgb", rgbMat);
// 		// }
// 		
// 		char key = cv::waitKey(5);
// 		
// 		if (key == 'x') {
// 			break;
// 		} else if (key == 's') {
// 			images.push_back(rgbMat.clone());
// 		}
// 	}
// 	
// 	device.stopVideo();
// 	
// 	std::cout << "Images: " << images.size() << std::endl;
// 	
// 	for (std::list<cv::Mat>::iterator iter = images.begin(); iter != images.end(); ++iter) {
// 		cv::imshow("rgb", *iter);
// 	
// 		cv::Mat greyMat(cv::Size(640, 480), CV_8UC1, cv::Scalar(0));
// 	
// 		cv::cvtColor(*iter, greyMat, CV_RGB2GRAY);
// 		std::vector<cv::Point2f> corners;
// 		
// 		bool found = cv::findChessboardCorners(greyMat, cv::Size(7, 7), corners);
// 		
// 		std::cout << "Corners found: " << corners.size() << std::endl;
// 	}
// 	
// 	cv::destroyWindow("depth");
// 	cv::destroyWindow("rgb");
// 	
// 	std::cout << "Exit" << std::endl;
// 	
// 	return 0;
// }