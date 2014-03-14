#pragma once

#include <opencv2/core/core.hpp>

class IUndistortedImageReceiver {
public:
	virtual void receiveUndistortedImage(cv::Mat *image, long int time) = 0;
};