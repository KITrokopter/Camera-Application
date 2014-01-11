#ifndef IMAGE_ANALYZER_HPP
#define IMAGE_ANALYZER_HPP

//#include <opencv2/core/mat.hpp>
#include "CvKinect.hpp"
#include "IImageReceiver.hpp"

class ImageAnalyzer : IImageReceiver {
private:
	cv::Mat* lastImage;
	CvKinect* imageSource;
	
public:
	ImageAnalyzer(CvKinect* imageSource);
	void start();
	void stop();
	cv::Mat* getImage();
	void receiveImage(cv::Mat* image);
	~ImageAnalyzer();
};

#endif // IMAGE_ANALYZER_HPP