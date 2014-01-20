#ifndef CV_IMAGE_PROCESSOR_HPP
#define CV_IMAGE_PROCESSOR_HPP

#include <vector>
#include <opencv2/core/core.hpp>
#include <boost/thread.hpp>

#include "ImageAnalyzer.hpp"
#include "Tracker.hpp"
#include "ITrackerDataReceiver.hpp"

class CvImageProcessor
{
private:
	ImageAnalyzer* imageAnalyzer;
	IImageReceiver* calibrationImageReceiver;
	ITrackerDataReceiver* dataReceiver;
	
	std::vector<cv::Scalar> colorRanges;
	cv::Mat* intrinsicsMatrix;
	cv::Mat* distortionCoefficients;
	std::vector<cv::Mat*> calibrationImages;
	
	// Camera calibration.
	boost::thread* calibrationThread;
	volatile bool abortCalibration;
	volatile double calibrationError;
	int imageAmount;
	int imageDelay;
	int boardWidth;
	int boardHeight;
	float boardRectangleWidth;
	float boardRectangleHeight;
	
	// Tracking
	std::vector<Tracker*> trackers;
	
	// Methods
	std::vector<cv::Point3f>* createObjectPoints();
	std::vector<cv::Point2f>* createImagePoints();
public:
	CvImageProcessor(ImageAnalyzer* imageAnalyzer, ITrackerDataReceiver* dataReceiver);
	
	void setIntrinsicsMatrix(cv::Mat* intrinsicsMatrix);
	void setDistortionCoefficients(cv::Mat* distortionCoefficients);
	
	// Camera calibration.
	void calibrateCamera();
	void clearCalibrationImages();
	void waitForCalibration();
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
	
	// Tracking
	void addQuadcopter(QuadcopterColor* qc);
	void removeQuadcopter(int id);
};

#endif // CV_IMAGE_PROCESSOR_HPP