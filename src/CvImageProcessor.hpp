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
	
	std::vector<cv::Scalar> colorRanges;
	cv::Mat* intrinsicsMatrix;
	cv::Mat* distortionCoefficients;
	std::vector<cv::Mat*> calibrationImages;
	
	// Camera calibration.
	boost::thread* calibrationThread;
	volatile bool abortCalibration;
	int imageAmount;
	int imageDelay;
	int boardWidth;
	int boardHeight;
	float boardRectangleWidth;
	float boardRectangleHeight;
public:
	CvImageProcessor(ImageAnalyzer* imageAnalyzer);
	
	void setIntrinsicsMatrix(cv::Mat* intrinsicsMatrix);
	void setDistortionCoefficients(cv::Mat* distortionCoefficients);
	
	// Camera calibration.
	void calibrateCamera();
	void clearCalibrationImages();
	
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
	void startCalibration(int imageAmount, int imageDelay, int boardWidth, int boardHeight, float boardRectangleWidth, float boardRectangleHeight);
};

#endif // CV_IMAGE_PROCESSOR_HPP