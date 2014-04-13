#include "libfreenect.hpp"
#include "ImageAnalyzer.hpp"

#include <ros/console.h>

/**
 * Creates a new ImageAnalyzer using the given CvKinect as image source.
 */
ImageAnalyzer::ImageAnalyzer(CvKinect *imageSource)
{
	this->imageSource = imageSource;
	imageSource->addImageReceiver(this);
	lastImage = 0;
	videoStarted = false;
	imageReceivedCount = 0;
}

/**
 * Starts the sending of bgr images by the CvKinect.
 */
void ImageAnalyzer::start()
{
	imageSource->startVideo();
	videoStarted = true;

	ROS_DEBUG("Video started");
}

/**
 * Stops the sending of bgr images by the CvKinect.
 */
void ImageAnalyzer::stop()
{
	imageSource->stopVideo();
	videoStarted = false;

	ROS_DEBUG("Video stopped");
}

/**
 * Returns if the image sending by the CvKinect is started.
 *
 * @return True if the CvKinect is currently sending images, false otherwise.
 */
bool ImageAnalyzer::isStarted()
{
	return videoStarted;
}

/**
 * Returns a shallow copy of the last image. The image matrix is the same as the
 * image matrix of the internal image.
 */
cv::Mat* ImageAnalyzer::getImage()
{
	if (lastImage == 0) {
		return 0;
	} else {
		imageMutex.lock();
		cv::Mat *result = new cv::Mat(*lastImage);
		imageMutex.unlock();

		return result;
	}
}

void ImageAnalyzer::receiveImage(cv::Mat *image, long int time, int type)
{
	imageMutex.lock();

	if (lastImage != 0)
		delete lastImage;

//  if (++imageReceivedCount % 150 == 0) {
//      ROS_DEBUG("Received %d images", imageReceivedCount);
//  }

	lastImage = image;
	lastImageTime = time;

	processImage(new cv::Mat(*image), time);

	imageMutex.unlock();
}

/**
 * Destroys the image analyzer, but doesn't stop the picture sending of
 * the CvKinect, if it was activated before.
 */
ImageAnalyzer::~ImageAnalyzer()
{
	imageSource->removeImageReceiver(this);

	if (lastImage != 0)
		delete lastImage;
}