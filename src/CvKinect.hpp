#ifndef CV_KINECT_HPP
#define CV_KINECT_HPP

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/console.h>

#include "libfreenect.hpp"
#include "Mutex.hpp"
#include "IImageReceiver.hpp"

/**
 * C++ interface for libfreenect.
 *
 * @author http://openkinect.org/wiki/C%2B%2BOpenCvExample
 * @author Sebastian Schmidt
 */
class CvKinect : public Freenect::FreenectDevice {
public:
	static const int KINECT_IMAGE_WIDTH = 640;
	static const int KINECT_IMAGE_HEIGHT = 480;

	CvKinect(freenect_context *_ctx, int _index);

	// Do not call directly even in child
	void VideoCallback(void *_rgb, uint32_t timestamp);

	// Do not call directly even in child
	void DepthCallback(void *_depth, uint32_t timestamp);

	bool getVideo(cv::Mat &output);

	bool getDepth(cv::Mat &output);

	void addImageReceiver(IImageReceiver *receiver);

	void removeImageReceiver(IImageReceiver *receiver);

private:
	std::vector<uint8_t> m_buffer_depth;
	std::vector<uint8_t> m_buffer_rgb;
	std::vector<uint16_t> m_gamma;
	cv::Mat depthMat;
	cv::Mat rgbMat;
	cv::Mat ownMat;
	Mutex m_rgb_mutex;
	Mutex m_depth_mutex;
	bool m_new_rgb_frame;
	bool m_new_depth_frame;

	int imageCount;
	long int imageCountTime;

	std::vector<IImageReceiver*> imageReceivers;
};

#endif // CV_KINECT_HPP
