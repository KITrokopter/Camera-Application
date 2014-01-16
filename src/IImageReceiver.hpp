#ifndef IIMAGE_RECEIVER_HPP
#define IIMAGE_RECEIVER_HPP

#include <opencv2/core/mat.hpp>

class IImageReceiver {
public:
	/*
	 * Function: receiveImage
	 * 
	 * Passes a copy of the received image.
	 */
	virtual void receiveImage(cv::Mat* image) = 0;
};

#endif // IIMAGE_RECEIVER_HPP