#ifndef IMAGE_ANALYZER_HPP
#define IMAGE_ANALYZER_HPP

#include <opencv2/core/core.hpp>
#include "CvKinect.hpp"
#include "IImageReceiver.hpp"
#include "Mutex.hpp"

class ImageAnalyzer : IImageReceiver
{
private:
	cv::Mat* lastImage;
	long int lastImageTime;
	CvKinect* imageSource;
	Mutex imageMutex;
	bool videoStarted;
	
public:
	ImageAnalyzer(CvKinect* imageSource);
	void start();
	void stop();
	bool isStarted();
	cv::Mat* getImage();
	void receiveImage(cv::Mat* image, long int time);
	
	~ImageAnalyzer();
};

#endif // IMAGE_ANALYZER_HPP
