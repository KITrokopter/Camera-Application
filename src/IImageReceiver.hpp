#ifndef IIMAGE_RECEIVER_HPP
#define IIMAGE_RECEIVER_HPP

#include <opencv2/core/mat.hpp>

class IImageReceiver {
public:
	virtual void receiveImage(cv::Mat* image) = 0;
};

#endif // IIMAGE_RECEIVER_HPP