#pragma once

#include <opencv2/core/core.hpp>

/**
 * Listener to receive undistorted images.
 *
 * @author Sebastian Schmidt
 */
class IUndistortedImageReceiver {
public:
	/**
	 * Passes a copy of the undistorted image.
	 * The receiver has to delete the object.
	 *
	 * @param image The undistorted image.
	 * @param time The time the image was taken.
	 */
	virtual void receiveUndistortedImage(cv::Mat *image, long int time) = 0;
};