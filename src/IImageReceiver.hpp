#ifndef IIMAGE_RECEIVER_HPP
#define IIMAGE_RECEIVER_HPP

#include <opencv2/core/core.hpp>

class IImageReceiver
{
public:
	/*
	 * Function: receiveImage
	 * 
	 * Passes a copy of the received image.
	 * The receiver has to delete the object.
	 */
	virtual void receiveImage(cv::Mat* image, long int time) = 0;
};

#endif // IIMAGE_RECEIVER_HPP
