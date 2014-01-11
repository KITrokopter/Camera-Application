#ifndef CV_KINECT_HPP
#define CV_KINECT_HPP

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "libfreenect.hpp"
#include "Mutex.hpp"
#include "IImageReceiver.hpp"

class CvKinect : public Freenect::FreenectDevice {


	public:
		CvKinect(freenect_context *_ctx, int _index);
		
		// Do not call directly even in child
		void VideoCallback(void* _rgb, uint32_t timestamp);
		
		// Do not call directly even in child
		void DepthCallback(void* _depth, uint32_t timestamp);
		
		bool getVideo(cv::Mat& output);
		
		bool getDepth(cv::Mat& output);
		
		void setImageReceiver(IImageReceiver* receiver);
		
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
		
		IImageReceiver* imageReceiver;
};

#endif // CV_KINECT_HPP
