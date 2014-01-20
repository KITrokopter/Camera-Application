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
	CvKinect* imageSource;
	Mutex imageMutex;
	bool videoStarted;
	std::vector<cv::Scalar> minColors;
	std::vector<cv::Scalar> maxColors;
	std::vector<int> quadcopterIds;
	
public:
	ImageAnalyzer(CvKinect* imageSource);
	void start();
	void stop();
	bool isStarted();
	cv::Mat* getImage();
	void receiveImage(cv::Mat* image);
	
	void addQuadcopter(cv::Scalar minColor, cv::Scalar maxColor, int id);
	void removeQuadcopter(int id);
	
	~ImageAnalyzer();
};

#endif // IMAGE_ANALYZER_HPP