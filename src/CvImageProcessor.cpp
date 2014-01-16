#include "CvImageProcessor.hpp"
#include <opencv/cv.h>

CvImageProcessor::CvImageProcessor(ImageAnalyzer* imageAnalyzer)
{
	this->intrinsicsMatrix = 0;
	this->distortionCoefficients = 0;
	this->calibrationThread = 0;
	this->imageAnalyzer = imageAnalyzer;
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
	std::vector<std::vector<cv::Point2f> > objectPoints; // TODO: Fill with content according to our calibration pattern (chessboard)
	cv::Mat intrinsicsMatrix(3, 3, CV_32FC1);
	cv::Mat distortionCoefficients(5, 1, CV_32FC1);
	
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
			// TODO: Good termination values? Good cv::Size values?
			cv::cornerSubPix(*image, corners, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 30, 0.1));
			imagePoints.push_back(corners);
			
			calibrationImages.push_back(image);
		} else {
			delete image;
		}
	}
	
	std::vector<std::vector<cv::Point2f> > rvecs;
	std::vector<std::vector<cv::Point2f> > tvecs;
	
	// Calibrate camera.
	// Eventually use CV_CALIB_FIX_K3, since it is only really useful for fisheye lenses.
	cv::calibrateCamera(objectPoints, imagePoints, cv::Size(CvKinect::KINECT_IMAGE_WIDTH, CvKinect::KINECT_IMAGE_HEIGHT), intrinsicsMatrix, distortionCoefficients, rvecs, tvecs, 0);
	
	setIntrinsicsMatrix(&intrinsicsMatrix);
	setDistortionCoefficients(&distortionCoefficients);
	
	// If video wasn't started before, stop it.
	if (!videoStarted) {
		imageAnalyzer->stop();
	}
}

void CvImageProcessor::clearCalibrationImages()
{
	while (calibrationImages.size() > 0) {
		delete calibrationImages.back();
		calibrationImages.pop_back();
	}
}

void CvImageProcessor::startCalibration(int imageAmount, int imageDelay, int boardWidth, int boardHeight, float boardRectangleWidth, float boardRectangleHeight)
{
	this->imageAmount = imageAmount;
	this->imageDelay = imageDelay;
	this->boardWidth = boardWidth;
	this->boardHeight = boardHeight;
	this->boardRectangleWidth = boardRectangleWidth;
	this->boardRectangleHeight = boardRectangleHeight;
	abortCalibration = false;
	
	setIntrinsicsMatrix(0);
	setDistortionCoefficients(0);
	
	calibrationThread = new boost::thread(boost::bind(&CvImageProcessor::calibrateCamera, this));
}