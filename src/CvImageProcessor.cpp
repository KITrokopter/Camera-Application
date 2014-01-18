#include "CvImageProcessor.hpp"
#include <opencv/cv.h>
#include <ros/console.h>

CvImageProcessor::CvImageProcessor(ImageAnalyzer* imageAnalyzer)
{
	this->intrinsicsMatrix = 0;
	this->distortionCoefficients = 0;
	this->calibrationThread = 0;
	this->imageAnalyzer = imageAnalyzer;
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

std::vector<cv::Point2f>* CvImageProcessor::createObjectPoints()
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
	std::vector<std::vector<cv::Point2f> > allObjectPoints;
	cv::Mat intrinsicsMatrix(3, 3, CV_32FC1);
	cv::Mat distortionCoefficients(4, 1, CV_32FC1);
	
	// Fill object points with data about the chessboard
	std::vector<cv::Point2f> objectPoints = *createObjectPoints();
	
	for (int i = 0; i < imageAmount; i++) {
		allObjectPoints.push_back(objectPoints);
	}
	
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
			ROS_DEBUG("Found good image");
			
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
				calibrationImageReceiver->receiveImage(new cv::Mat(*image));
			}
		} else {
			ROS_DEBUG("Found bad image");
			delete image;
		}
	}
	
	std::vector<std::vector<cv::Point2f> > rvecs;
	std::vector<std::vector<cv::Point2f> > tvecs;
	
	// Calibrate camera.
	// Use CV_CALIB_FIX_K3, since k3 is only really useful for fisheye lenses.
	cv::calibrateCamera(allObjectPoints, imagePoints, cv::Size(CvKinect::KINECT_IMAGE_WIDTH, CvKinect::KINECT_IMAGE_HEIGHT), intrinsicsMatrix, distortionCoefficients, rvecs, tvecs, CV_CALIB_FIX_K3, cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 5, DBL_EPSILON));
	
	setIntrinsicsMatrix(&intrinsicsMatrix);
	setDistortionCoefficients(&distortionCoefficients);
	
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
	free(calibrationThread);
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