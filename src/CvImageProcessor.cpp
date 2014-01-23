#include "CvImageProcessor.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <ros/console.h>
#include <iostream>

CvImageProcessor::CvImageProcessor(ImageAnalyzer* imageAnalyzer, ITrackerDataReceiver* dataReceiver)
{
	this->intrinsicsMatrix = 0;
	this->distortionCoefficients = 0;
	this->calibrationThread = 0;
	this->imageAnalyzer = imageAnalyzer;
	this->dataReceiver = dataReceiver;
	this->calibrationImageReceiver = 0;
}

void CvImageProcessor::setIntrinsicsMatrix(cv::Mat* intrinsicsMatrix)
{
	if (this->intrinsicsMatrix != 0) {
		delete this->intrinsicsMatrix;
	}
	
	this->intrinsicsMatrix = intrinsicsMatrix;
}

void CvImageProcessor::setDistortionCoefficients(cv::Mat* distortionCoefficients)
{
	if (this->distortionCoefficients != 0) {
		delete this->distortionCoefficients;
	}
	
	this->distortionCoefficients = distortionCoefficients;
}

std::vector<cv::Point3f>* CvImageProcessor::createObjectPoints()
{
	std::vector<cv::Point3f>* result = new std::vector<cv::Point3f>();
	
	for (int i = 0; i < boardWidth; i++) {
		for (int j = 0; j < boardHeight; j++) {
			cv::Point3f point(i * boardRectangleWidth, j * boardRectangleHeight, 0);
			result->push_back(point);
		}
	}
	
	return result;
}

std::vector<cv::Point2f>* CvImageProcessor::createImagePoints()
{
	std::vector<cv::Point2f>* result = new std::vector<cv::Point2f>();
	
	for (int i = 0; i < boardWidth; i++) {
		for (int j = 0; j < boardHeight; j++) {
			cv::Point2f point(i * boardRectangleWidth, j * boardRectangleHeight);
			result->push_back(point);
		}
	}
	
	return result;
}

void CvImageProcessor::calibrateCamera()
{
	bool videoStarted = imageAnalyzer->isStarted();
	
	if (!videoStarted) {
		imageAnalyzer->start();
	}
	
	cv::Size boardSize(boardWidth, boardHeight);
	int boardCornerCount = boardWidth * boardHeight;
	
	// Allocate storage.
	std::vector<std::vector<cv::Point2f> > imagePoints;
	std::vector<std::vector<cv::Point3f> > allObjectPoints;
	cv::Mat intrinsicsMatrix = cv::Mat::eye(3, 3, CV_32F);
	cv::Mat distortionCoefficients = cv::Mat::zeros(5, 1, CV_32F);
	
	// Fill object points with data about the chessboard
	std::vector<cv::Point3f>* objectPoints = createObjectPoints();
	
	for (int i = 0; i < imageAmount; i++) {
		allObjectPoints.push_back(*objectPoints);
	}
	
	// temporary create custom image points used for testing
	/*for (int i = 0; i < imageAmount; i++) {
		imagePoints.push_back(*createImagePoints());
	}*/
	
	int successfulImageAmount = 0;
	
	// Loop until we got enough images for calibration.
	while (successfulImageAmount < imageAmount) {
		// Wait a bit to give the user time to move the chessboard.
		usleep(1000 * imageDelay);
		
		// Get image.
		cv::Mat* image = imageAnalyzer->getImage();
		
		// Find chessboard corners.
		std::vector<cv::Point2f> corners;
		bool foundAllCorners = cv::findChessboardCorners(*image, boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		
		if (foundAllCorners) {
			std::cout << "Found good image" << std::endl;
			
			cv::Mat greyImage(image->size(), CV_8UC1);
			cv::cvtColor(*image, greyImage, CV_RGB2GRAY);
			
			// TODO: Good termination values? Good cv::Size values?
			cv::cornerSubPix(greyImage, corners, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 30, 0.1));
			imagePoints.push_back(corners);
			
			// Add information about the chessboard found.
			cv::drawChessboardCorners(*image, boardSize, corners, true);
			
			// Save image.
			calibrationImages.push_back(image);
			
			// Notify listener about new image.
			if (calibrationImageReceiver != 0) {
				calibrationImageReceiver->receiveImage(new cv::Mat(*image), -1);
			}
			
			successfulImageAmount++;
		} else {
			std::cout << "Found bad image" << std::endl;
			delete image;
		}
	}
	
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
	
	std::cout << "Start calibration calculation" << std::endl;
	
	// Calibrate camera.
	// Use CV_CALIB_FIX_K3, since k3 is only really useful for fisheye lenses.
	calibrationError = cv::calibrateCamera(allObjectPoints, imagePoints, cv::Size(CvKinect::KINECT_IMAGE_WIDTH, CvKinect::KINECT_IMAGE_HEIGHT), intrinsicsMatrix, distortionCoefficients, rvecs, tvecs, CV_CALIB_FIX_K3, cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, DBL_EPSILON));
	
	setIntrinsicsMatrix(&intrinsicsMatrix);
	setDistortionCoefficients(&distortionCoefficients);
	std::cout << "Calibration successful" << std::endl;
	
	// Free object description
	delete objectPoints;
	
	// If video wasn't started before, stop it.
	if (!videoStarted) {
		imageAnalyzer->stop();
	}
	
	calibrationImageReceiver = 0;
}

void CvImageProcessor::clearCalibrationImages()
{
	while (calibrationImages.size() > 0) {
		delete calibrationImages.back();
		calibrationImages.pop_back();
	}
}

void CvImageProcessor::waitForCalibration()
{
	calibrationThread->join();
	delete calibrationThread;
	calibrationThread = 0;
}

cv::Mat* CvImageProcessor::undistortImage(cv::Mat* inputImage) {
	cv::Mat* undistorted = new cv::Mat(inputImage->size(), inputImage->type());
	cv::undistort(*inputImage, *undistorted, *intrinsicsMatrix, *distortionCoefficients);
}

void CvImageProcessor::startCalibration(int imageAmount, int imageDelay, int boardWidth, int boardHeight, float boardRectangleWidth, float boardRectangleHeight, IImageReceiver* calibrationImageReceiver)
{
	this->imageAmount = imageAmount;
	this->imageDelay = imageDelay;
	this->boardWidth = boardWidth;
	this->boardHeight = boardHeight;
	this->boardRectangleWidth = boardRectangleWidth;
	this->boardRectangleHeight = boardRectangleHeight;
	this->calibrationImageReceiver = calibrationImageReceiver;
	abortCalibration = false;
	
	setIntrinsicsMatrix(0);
	setDistortionCoefficients(0);
	
	calibrationThread = new boost::thread(boost::bind(&CvImageProcessor::calibrateCamera, this));
}
