#include "CvImageProcessor.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <ros/console.h>
#include <iostream>

CvImageProcessor::CvImageProcessor(CvKinect* imageSource, ITrackerDataReceiver* dataReceiver)
		: ImageAnalyzer::ImageAnalyzer(imageSource)
{
	this->intrinsicsMatrix = 0;
	this->distortionCoefficients = 0;
	this->calibrationThread = 0;
	this->dataReceiver = dataReceiver;
	this->calibrationImageReceiver = 0;
	this->isTracking = false;
}

void CvImageProcessor::setIntrinsicsMatrix(cv::Mat* intrinsicsMatrix)
{
	if (this->intrinsicsMatrix != 0) {
		delete this->intrinsicsMatrix;
	}
	
	if (intrinsicsMatrix != 0) {
		this->intrinsicsMatrix = new cv::Mat(*intrinsicsMatrix);
	} else {
		this->intrinsicsMatrix = 0;
	}
}

void CvImageProcessor::setDistortionCoefficients(cv::Mat* distortionCoefficients)
{
	if (this->distortionCoefficients != 0) {
		delete this->distortionCoefficients;
	}
	
	if (distortionCoefficients != 0) {
		this->distortionCoefficients = new cv::Mat(*distortionCoefficients);
	} else {
		this->distortionCoefficients = 0;
	}
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
	bool videoStarted = isStarted();
	
	if (!videoStarted) {
		start();
	}
	
	cv::Size boardSize(boardWidth, boardHeight);
	int boardCornerCount = boardWidth * boardHeight;
	
	// Allocate storage.
	std::vector<std::vector<cv::Point2f> > imagePoints;
	std::vector<std::vector<cv::Point3f> > allObjectPoints;
	cv::Mat intrinsicsMatrix = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat distortionCoefficients = cv::Mat::zeros(5, 1, CV_64F);
	
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
	while (successfulImageAmount < imageAmount && !abortCalibrationFlag) {
		// Wait a bit to give the user time to move the chessboard.
		usleep(1000 * imageDelay);
		
		// Get image.
		cv::Mat* image = getImage();
		
		// Find chessboard corners.
		std::vector<cv::Point2f> corners;
		bool foundAllCorners = cv::findChessboardCorners(*image, boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
		
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
				calibrationImageReceiver->receiveImage(new cv::Mat(*image), successfulImageAmount, 1);
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
	calibrationError = cv::calibrateCamera(allObjectPoints, imagePoints, cv::Size(CvKinect::KINECT_IMAGE_WIDTH, CvKinect::KINECT_IMAGE_HEIGHT), intrinsicsMatrix, distortionCoefficients, rvecs, tvecs, CV_CALIB_FIX_K3 | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5 | CV_CALIB_FIX_K6 | CV_CALIB_USE_INTRINSIC_GUESS, cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 300, DBL_EPSILON));
	
	setIntrinsicsMatrix(&intrinsicsMatrix);
	setDistortionCoefficients(&distortionCoefficients);
	std::cout << "Calibration successful" << std::endl;
	ROS_DEBUG("Error: %f, Target: %f", calibrationError, DBL_EPSILON);
	
	if (calibrationFinishedListener != 0) {
		calibrationFinishedListener->calibrationFinished(&intrinsicsMatrix, &distortionCoefficients);
	}
	
	// Free object description
	delete objectPoints;
	
	// If video wasn't started before, stop it.
	if (!videoStarted) {
		stop();
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
	return undistorted;
}

void CvImageProcessor::startCalibration(int imageAmount, int imageDelay, int boardWidth, int boardHeight, float boardRectangleWidth, float boardRectangleHeight, IImageReceiver* calibrationImageReceiver, ICalibrationFinishedListener* calibrationFinishedListener)
{
	this->imageAmount = imageAmount;
	this->imageDelay = imageDelay;
	this->boardWidth = boardWidth;
	this->boardHeight = boardHeight;
	this->boardRectangleWidth = boardRectangleWidth;
	this->boardRectangleHeight = boardRectangleHeight;
	this->calibrationImageReceiver = calibrationImageReceiver;
	this->calibrationFinishedListener = calibrationFinishedListener;
	abortCalibrationFlag = false;
	
	setIntrinsicsMatrix(0);
	setDistortionCoefficients(0);
	
	calibrationThread = new boost::thread(boost::bind(&CvImageProcessor::calibrateCamera, this));
}

void CvImageProcessor::processImage(cv::Mat* image, long int time)
{
	if (isTracking) {
		ROS_DEBUG("Processing image");
		
		cv::Mat* undistorted = undistortImage(image);
		
		ROS_DEBUG("Image successfully undistorted");
		
		for (std::vector<Tracker*>::iterator it = trackers.begin(); it != trackers.end(); it++) {
			(*it)->setNextImage(undistorted, time);
		}
		
		ROS_DEBUG("Processing image shared between %ld threads", trackers.size());
	}
	
	delete image;
}

void CvImageProcessor::startTracking()
{
	for (std::vector<Tracker*>::iterator it = trackers.begin(); it != trackers.end(); it++) {
		(*it)->start();
	}
	
	isTracking = true;
	
	ROS_DEBUG("Tracking started");
}

void CvImageProcessor::stopTracking()
{
	isTracking = false;
	
	for (std::vector<Tracker*>::iterator it = trackers.begin(); it != trackers.end(); it++) {
		(*it)->stop();
	}
	
	for (std::vector<Tracker*>::iterator it = trackers.begin(); it != trackers.end(); it++) {
		(*it)->join();
	}
}

void CvImageProcessor::setDataReceiver(ITrackerDataReceiver* receiver)
{
	this->dataReceiver = receiver;
}

void CvImageProcessor::addQuadcopter(QuadcopterColor* qc)
{
	Tracker* tracker = new Tracker(dataReceiver, qc);
	trackers.push_back(tracker);
}

void CvImageProcessor::removeQuadcopter(int id)
{
	for (std::vector<Tracker*>::iterator it = trackers.begin(); it != trackers.end(); it++) {
		if ((*it)->getQuadcopterColor()->getId() == id) {
			delete *it;
			trackers.erase(it);
		}
	}
}

cv::Mat* CvImageProcessor::getIntrinsicsMatrix()
{
	return new cv::Mat(*intrinsicsMatrix);
}

cv::Mat* CvImageProcessor::getDistortionCoefficients()
{
	return new cv::Mat(*distortionCoefficients);
}

void CvImageProcessor::abortCalibration()
{
	abortCalibrationFlag = true;
}

bool CvImageProcessor::isCalibrated()
{
	return intrinsicsMatrix != 0 && distortionCoefficients != 0;
}

CvImageProcessor::~CvImageProcessor()
{
}