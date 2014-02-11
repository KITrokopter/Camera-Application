#ifndef IMAGE_ANALYZER_HPP
#define IMAGE_ANALYZER_HPP

#include <opencv2/core/core.hpp>
#include "CvKinect.hpp"
#include "IImageReceiver.hpp"
#include "Mutex.hpp"

class ImageAnalyzer : public IImageReceiver
{
private:
	cv::Mat* lastImage;
	long int lastImageTime;
	Mutex imageMutex;
	bool videoStarted;
	CvKinect* imageSource;
	
protected:
	virtual void processImage(cv::Mat* image, long int time) = 0;
	
public:
	ImageAnalyzer(CvKinect* imageSource);
	void start();
	void stop();
	bool isStarted();
	cv::Mat* getImage();
	void receiveImage(cv::Mat* image, long int time, int type);
	
	~ImageAnalyzer();
};

#endif // IMAGE_ANALYZER_HPP
