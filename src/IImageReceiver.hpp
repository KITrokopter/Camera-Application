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
	 * 
	 * Parameters:
	 *   image* - The image.
	 *   time - The time the image was taken.
	 *   type - 0 for normal image, 1 for calibration image.
	 */
	virtual void receiveImage(cv::Mat* image, long int time, int type) = 0;
};

#endif // IIMAGE_RECEIVER_HPP
