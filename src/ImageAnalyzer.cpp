#include "libfreenect.hpp"
#include "ImageAnalyzer.hpp"

#include <ros/console.h>

ImageAnalyzer::ImageAnalyzer(CvKinect* imageSource)
{
	this->imageSource = imageSource;
	imageSource->addImageReceiver(this);
	lastImage = 0;
	videoStarted = false;
}

void ImageAnalyzer::start()
{
	imageSource->startVideo();
	videoStarted = true;
	
	ROS_DEBUG("Video started");
}

void ImageAnalyzer::stop()
{
	imageSource->stopVideo();
	videoStarted = false;
	
	ROS_DEBUG("Video stopped");
}

bool ImageAnalyzer::isStarted()
{
	return videoStarted;
}

// Returns a shallow copy of the last image. The image matrix is the same as the image matrix of the internal image.
cv::Mat* ImageAnalyzer::getImage()
{
	if (lastImage == 0) {
		return 0;
	} else {
		imageMutex.lock();
		cv::Mat* result = new cv::Mat(*lastImage);
		imageMutex.unlock();
		
		return result;
	}
}

void ImageAnalyzer::receiveImage(cv::Mat* image, long int time, int type)
{
	imageMutex.lock();
	
	if (lastImage != 0) {
		delete lastImage;
	}
	
	lastImage = image;
	lastImageTime = time;
	
	processImage(new cv::Mat(*image), time);
	
	imageMutex.unlock();
}

ImageAnalyzer::~ImageAnalyzer()
{
	imageSource->removeImageReceiver(this);
	
	if (lastImage != 0) {
		delete lastImage;
	}
}