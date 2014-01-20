#ifndef CV_IMAGE_PROCESSOR_HPP
#define CV_IMAGE_PROCESSOR_HPP

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <boost/thread.hpp>

#include "ImageAnalyzer.hpp"

class CvImageProcessor {
private:
	ImageAnalyzer* imageAnalyzer;
	IImageReceiver* calibrationImageReceiver;
	
	std::vector<cv::Scalar> colorRanges;
	cv::Mat* intrinsicsMatrix;
	cv::Mat* distortionCoefficients;
	std::vector<cv::Mat*> calibrationImages;
	
	// Camera calibration.
	boost::thread* calibrationThread;
	volatile bool abortCalibration;
	volatile bool calibrationError;
	int imageAmount;
	int imageDelay;
	int boardWidth;
	int boardHeight;
	float boardRectangleWidth;
	float boardRectangleHeight;
	
	std::vector<cv::Point3f>* createObjectPoints();
	std::vector<cv::Point2f>* createImagePoints(); // TODO remove
public:
	CvImageProcessor(ImageAnalyzer* imageAnalyzer);
	
	void setIntrinsicsMatrix(cv::Mat* intrinsicsMatrix);
	void setDistortionCoefficients(cv::Mat* distortionCoefficients);
	
	// Camera calibration.
	void calibrateCamera();
	void clearCalibrationImages();
	bool waitForCalibration();
	cv::Mat* undistortImage(cv::Mat* inputImage);
	
	/*
	 * Method: startCalibration
	 * 
	 * Starts the camera calibration process.
	 * 
	 * Parameters:
	 * imageAmount - The amount of images to use for the calibration.
	 * imageDelay - The delay between two images.
	 * boardWidth - The amount of corners on the x axis of the board.
	 * boardHeight - The amount of corners on the y axis of the board.
	 * boardRectangleWidth - The width of one board rectangle.
	 * boardRectangleHeight - The height of one board rectangle.
	 */
	void startCalibration(int imageAmount, int imageDelay, int boardWidth, int boardHeight, float boardRectangleWidth, float boardRectangleHeight, IImageReceiver* calibrationImageReceiver);
};

#endif // CV_IMAGE_PROCESSOR_HPP