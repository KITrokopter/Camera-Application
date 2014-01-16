#ifndef IMAGE_ANALYZER_HPP
#define IMAGE_ANALYZER_HPP

//#include <opencv2/core/mat.hpp>
#include "CvKinect.hpp"
#include "IImageReceiver.hpp"
#include "Mutex.hpp"

class ImageAnalyzer : IImageReceiver
{
private:
	cv::Mat* lastImage;
	CvKinect* imageSource;
	Mutex imageMutex;
	bool videoStarted;
	
public:
	ImageAnalyzer(CvKinect* imageSource);
	void start();
	void stop();
	bool isStarted();
	cv::Mat* getImage();
	void receiveImage(cv::Mat* image);
	
	~ImageAnalyzer();
};

#endif // IMAGE_ANALYZER_HPP