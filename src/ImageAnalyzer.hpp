#ifndef IMAGEANALYZER_HPP
#define IMAGEANALYZER_HPP

#include <opencv/core/mat.hpp>
#include "CvKinect.hpp"
#include "IImageReceiver.hpp"

class ImageAnalyzer : IImageReceiver {
private:
	Mat* lastImage;
	CvKinect* imageSource;
	
public:
	ImageAnalyzer(CvKinect* imageSource);
	void start();
	void stop();
	Mat* getImage();
	void receiveImage(Mat* image);
	~ImageAnalyzer();
};

#endif // IMAGEANALYZER_HPP