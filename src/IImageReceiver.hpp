#ifndef IIMAGE_RECEIVER_HPP
#define IIMAGE_RECEIVER_HPP

#include <opencv2/core/core.hpp>

/**
 * Listener that receives images from the Kinect.
 *
 * @author Sebastian Schmidt
 */
class IImageReceiver {
public:
	/**
	 * Passes a copy of the received image.
	 * The receiver has to delete the image object.
	 *
	 * @param image The image.
	 * @param time The time the image was taken.
	 * @param type 0 for normal image, 1 for calibration image.
	 */
	virtual void receiveImage(cv::Mat *image, long int time, int type) = 0;
};

#endif // IIMAGE_RECEIVER_HPP
