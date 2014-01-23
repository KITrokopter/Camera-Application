#include "libfreenect.hpp"
#include "ImageAnalyzer.hpp"

ImageAnalyzer::ImageAnalyzer(CvKinect* imageSource)
{
	this->imageSource = imageSource;
	imageSource->addImageReceiver(this);
	lastImage = 0;
	videoStarted = false;
}

void ImageAnalyzer::start()
{
	imageSource->startVideo();
	videoStarted = true;
}

void ImageAnalyzer::stop()
{
	imageSource->stopVideo();
	videoStarted = false;
}

bool ImageAnalyzer::isStarted()
{
	return videoStarted;
}

// Returns a shallow copy of the last image. The image matrix is the same as the image matrix of the internal image.
cv::Mat* ImageAnalyzer::getImage()
{
	if (lastImage == 0) {
		return 0;
	} else {
		imageMutex.lock();
		cv::Mat* result = new cv::Mat(*lastImage);
		imageMutex.unlock();
		
		return result;
	}
}

void ImageAnalyzer::receiveImage(cv::Mat* image, long int time)
{
	imageMutex.lock();
	
	if (lastImage != 0) {
		delete lastImage;
	}
	
	lastImage = image;
	lastImageTime = time;
	
	imageMutex.unlock();
}

ImageAnalyzer::~ImageAnalyzer()
{
	imageSource->removeImageReceiver(this);
	
	if (lastImage != 0) {
		delete lastImage;
	}
}

// int main(void) {	
// 	cv::namedWindow("rgb", CV_WINDOW_AUTOSIZE);
// 	cv::namedWindow("depth", CV_WINDOW_AUTOSIZE);
// 	
// 	while (true) {
// 		device.getVideo(rgbMat);
// 		
// 		// if (images.size() > 0) {
// 		// 	cv::imshow("rgb", images.front());
// 		// } else {
// 			cv::imshow("rgb", rgbMat);
// 		// }
// 		
// 		char key = cv::waitKey(5);
// 		
// 		if (key == 'x') {
// 			break;
// 		} else if (key == 's') {
// 			images.push_back(rgbMat.clone());
// 		}
// 	}
// 	
// 	device.stopVideo();
// 	
// 	std::cout << "Images: " << images.size() << std::endl;
// 	
// 	for (std::list<cv::Mat>::iterator iter = images.begin(); iter != images.end(); ++iter) {
// 		cv::imshow("rgb", *iter);
// 	
// 		cv::Mat greyMat(cv::Size(640, 480), CV_8UC1, cv::Scalar(0));
// 	
// 		cv::cvtColor(*iter, greyMat, CV_RGB2GRAY);
// 		std::vector<cv::Point2f> corners;
// 		
// 		bool found = cv::findChessboardCorners(greyMat, cv::Size(7, 7), corners);
// 		
// 		std::cout << "Corners found: " << corners.size() << std::endl;
// 	}
// 	
// 	cv::destroyWindow("depth");
// 	cv::destroyWindow("rgb");
// 	
// 	std::cout << "Exit" << std::endl;
// 	
// 	return 0;
// }
