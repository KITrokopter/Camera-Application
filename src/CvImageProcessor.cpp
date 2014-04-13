#include "CvImageProcessor.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <ros/console.h>
#include <iostream>

/**
 * Creates a new object. The visual tracking options are disabled using this
 * constructor.
 *
 * @param imageSource A Kinect connection.
 * @param dataReceiver An object to send the calculated quadcopter positions
 * to.
 */
CvImageProcessor::CvImageProcessor(CvKinect *imageSource, ITrackerDataReceiver *dataReceiver)
	: ImageAnalyzer::ImageAnalyzer(imageSource)
{
	this->intrinsicsMatrix = 0;
	this->distortionCoefficients = 0;
	this->calibrationThread = 0;
	this->dataReceiver = dataReceiver;
	this->calibrationImageReceiver = 0;
	this->undistortedImageReceiver = 0;
	this->isTracking = false;
	this->showCameraImage = false;
	this->showMaskedImage = false;
}

/**
 * Creates a new object.
 *
 * @param imageSource A Kinect connection.
 * @param dataReceiver An object to send the calculated quadcopter positions
 * to.
 * @param showCameraImage True if the camera image should be shown during
 * tracking. (Resource intensive and needs $DISPLAY to be set.)
 * @param showMaskedImage True if the color masked image should be shown
 * during tracking. (Resource intensive and needs $DISPLAY to be set.)
 */
CvImageProcessor::CvImageProcessor(CvKinect *imageSource, ITrackerDataReceiver *dataReceiver, bool showCameraImage,
                                   bool showMaskedImage)
	: ImageAnalyzer::ImageAnalyzer(imageSource)
{
	this->intrinsicsMatrix = 0;
	this->distortionCoefficients = 0;
	this->calibrationThread = 0;
	this->dataReceiver = dataReceiver;
	this->calibrationImageReceiver = 0;
	this->undistortedImageReceiver = 0;
	this->isTracking = false;
	this->showCameraImage = showCameraImage;
	this->showMaskedImage = showMaskedImage;
}

/**
 * Sets the intrinsics matrix for the undistortion of the camera images. The
 * matrix will be copied internally.
 *
 * @param intrinsicsMatrix A 3x3 matrix containing double values.
 */
void CvImageProcessor::setIntrinsicsMatrix(cv::Mat *intrinsicsMatrix)
{
	if (this->intrinsicsMatrix != 0)
		delete this->intrinsicsMatrix;

	if (intrinsicsMatrix != 0)
		this->intrinsicsMatrix = new cv::Mat(*intrinsicsMatrix);
	else
		this->intrinsicsMatrix = 0;
}

/**
 * Sets the distortion coefficients for the undistortion of the camera
 * images. The matrix will be copied internally.
 *
 * @param distortionCoefficients A 5x1 matrix containing double values.
 */
void CvImageProcessor::setDistortionCoefficients(cv::Mat *distortionCoefficients)
{
	if (this->distortionCoefficients != 0)
		delete this->distortionCoefficients;

	if (distortionCoefficients != 0)
		this->distortionCoefficients = new cv::Mat(*distortionCoefficients);
	else
		this->distortionCoefficients = 0;
}

std::vector<cv::Point3f>* CvImageProcessor::createObjectPoints()
{
	std::vector<cv::Point3f> *result = new std::vector<cv::Point3f>();

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
	std::vector<cv::Point2f> *result = new std::vector<cv::Point2f>();

	for (int i = 0; i < boardWidth; i++) {
		for (int j = 0; j < boardHeight; j++) {
			cv::Point2f point(i * boardRectangleWidth, j * boardRectangleHeight);
			result->push_back(point);
		}
	}

	return result;
}

/**
 * The function that calibrates the camera. You can use {@link
 * startCalibration()} if you want a non-blocking method.
 */
void CvImageProcessor::calibrateCamera()
{
	clearCalibrationImages();

	bool videoStarted = isStarted();

	if (!videoStarted)
		start();

	cv::Size boardSize(boardWidth, boardHeight);
	int boardCornerCount = boardWidth * boardHeight;

	// Allocate storage.
	std::vector<std::vector<cv::Point2f> > imagePoints;
	std::vector<std::vector<cv::Point3f> > allObjectPoints;
	cv::Mat intrinsicsMatrix = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat distortionCoefficients = cv::Mat::zeros(5, 1, CV_64F);

	// Fill object points with data about the chessboard
	std::vector<cv::Point3f> *objectPoints = createObjectPoints();

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
		cv::Mat *image = getImage();

		// Find chessboard corners.
		std::vector<cv::Point2f> corners;
		bool foundAllCorners = cv::findChessboardCorners(
		    *image, boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS |
		    CV_CALIB_CB_FAST_CHECK |
		    CV_CALIB_CB_NORMALIZE_IMAGE);

		if (foundAllCorners) {
			std::cout << "Found good image" << std::endl;

			cv::Mat greyImage(image->size(), CV_8UC1);
			cv::cvtColor(*image, greyImage, CV_RGB2GRAY);

			// TODO: Good termination values? Good cv::Size values?
			cv::cornerSubPix(greyImage, corners,
			                 cv::Size(5,
			                          5),
			                 cv::Size(-1,
			                          -1), cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 30, 0.1));
			imagePoints.push_back(corners);

			// Add information about the chessboard found.
			cv::drawChessboardCorners(*image, boardSize, corners, true);

			// Save image.
			calibrationImages.push_back(image);

			// Notify listener about new image.
			if (calibrationImageReceiver != 0)
				calibrationImageReceiver->receiveImage(new cv::Mat(*image), successfulImageAmount, 1);

			successfulImageAmount++;
		} else {
			std::cout << "Found bad image" << std::endl;
			delete image;
		}
	}

	if (!abortCalibrationFlag) {
		std::vector<cv::Mat> rvecs;
		std::vector<cv::Mat> tvecs;

		std::cout << "Start calibration calculation" << std::endl;

		// Calibrate camera.
		// Use CV_CALIB_FIX_K3, since k3 is only really useful for fisheye
		// lenses.
		calibrationError =
		    cv::calibrateCamera(allObjectPoints, imagePoints,
		                        cv::Size(CvKinect::KINECT_IMAGE_WIDTH,
		                                 CvKinect::KINECT_IMAGE_HEIGHT), intrinsicsMatrix, distortionCoefficients,
		                        rvecs, tvecs, CV_CALIB_FIX_K3 |
		                        CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5 | CV_CALIB_FIX_K6 | CV_CALIB_USE_INTRINSIC_GUESS,
		                        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 300, DBL_EPSILON));

		setIntrinsicsMatrix(&intrinsicsMatrix);
		setDistortionCoefficients(&distortionCoefficients);
		std::cout << "Calibration successful" << std::endl;
		ROS_DEBUG("Error: %f, Target: %f", calibrationError, DBL_EPSILON);
	}

	if (calibrationFinishedListener != 0)
		calibrationFinishedListener->calibrationFinished(getIntrinsicsMatrix(), getDistortionCoefficients());

	// Free object description
	delete objectPoints;

	// If video wasn't started before, stop it.
	if (!videoStarted)
		stop();

	calibrationImageReceiver = 0;
}

/**
 * Deletes all images that were collected during calibration.
 */
void CvImageProcessor::clearCalibrationImages()
{
	while (calibrationImages.size() > 0) {
		delete calibrationImages.back();
		calibrationImages.pop_back();
	}
}

/**
 * Waits for the calibration to finish. Only works, if the
 * {@link startCalibration} method was used.
 */
void CvImageProcessor::waitForCalibration()
{
	calibrationThread->join();
	delete calibrationThread;
	calibrationThread = 0;
}

/**
 * Returns an undistorted version of the given image.<br />
 * Doesn't work if the camera wasn't calibrated before and no custom
 * calibration data has  been set via the {@link setIntrinsicsMatrix}
 * and {@link setDistortionCoefficients} methods.
 * The caller has to delete the result.
 *
 * @param inputImage The image to undistort.
 */
cv::Mat* CvImageProcessor::undistortImage(cv::Mat *inputImage)
{
	cv::Mat *undistorted = new cv::Mat(inputImage->size(), inputImage->type());
	cv::undistort(*inputImage, *undistorted, *intrinsicsMatrix, *distortionCoefficients);
	return undistorted;
}

/**
 * Starts the camera calibration process.
 *
 * @param imageAmount The amount of images to use for the calibration.
 * @param imageDelay The delay between two images.
 * @param boardWidth The amount of corners on the x axis of the board.
 * @param boardHeight The amount of corners on the y axis of the board.
 * @param boardRectangleWidth The width of one board rectangle.
 * @param boardRectangleHeight The height of one board rectangle.
 */
void CvImageProcessor::startCalibration(int imageAmount, int imageDelay, int boardWidth, int boardHeight,
                                        float boardRectangleWidth, float boardRectangleHeight,
                                        IImageReceiver *calibrationImageReceiver,
                                        ICalibrationFinishedListener *calibrationFinishedListener)
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

void CvImageProcessor::processImage(cv::Mat *image, long int time)
{
	cv::Mat *undistorted = 0;

	if (isCalibrated() && (isTracking || undistortedImageReceiver != 0))
		undistorted = undistortImage(image);

	if (isTracking)
		for (std::vector<Tracker*>::iterator it = trackers.begin(); it != trackers.end(); it++) {
			(*it)->setNextImage(undistorted, time);
		}

		// ROS_DEBUG("Processing image shared between %ld threads",
		// trackers.size());

	if (undistortedImageReceiver != 0) {
		cv::Mat *toSend = new cv::Mat(image->size(), image->type());
		image->copyTo(*toSend);

		undistortedImageReceiver->receiveUndistortedImage(toSend, time);
	}

	if (undistorted != 0)
		delete undistorted;

	delete image;
}

/**
 * Starts the tracking process.
 */
void CvImageProcessor::startTracking()
{
	if (isTracking)
		return;

	if (!isCalibrated())
		ROS_ERROR("Cannot start tracking! Not calibrated!");

	for (std::vector<Tracker*>::iterator it = trackers.begin(); it != trackers.end(); it++) {
		(*it)->start();
	}

	isTracking = true;

	ROS_DEBUG("Tracking started");
}

/**
 * Stops the tracking process.
 */
void CvImageProcessor::stopTracking()
{
	if (!isTracking)
		return;

	ROS_DEBUG("Stopping tracking...");

	isTracking = false;

	for (std::vector<Tracker*>::iterator it = trackers.begin(); it != trackers.end(); it++) {
		(*it)->stop();
	}

	ROS_DEBUG("Joining trackers");

	for (std::vector<Tracker*>::iterator it = trackers.begin(); it != trackers.end(); it++) {
		(*it)->join();
	}

	ROS_DEBUG("Tracking stopped");
}

/**
 * Sets the receiver for the calculated quadcopter vectors.
 *
 * @param receiver The receiver.
 */
void CvImageProcessor::setDataReceiver(ITrackerDataReceiver *receiver)
{
	this->dataReceiver = receiver;
}

/**
 * Adds a quadcopter to be tracked.
 *
 * @param addQuadcopter The quadcopter.
 */
void CvImageProcessor::addQuadcopter(QuadcopterColor *qc)
{
	Tracker *tracker = new Tracker(dataReceiver, qc, showCameraImage, showMaskedImage);
	trackers.push_back(tracker);
}

/**
 * Removes the quadcopter with the given id.
 *
 * @param id The id.
 */
void CvImageProcessor::removeQuadcopter(int id)
{
	for (std::vector<Tracker*>::iterator it = trackers.begin(); it != trackers.end(); it++) {
		if ((*it)->getQuadcopterColor()->getId() == id) {
			delete *it;
			trackers.erase(it);
			return;
		}
	}
}

/**
 * Removes all quadcopters.
 */
void CvImageProcessor::removeAllQuadcopters()
{
	while (trackers.size() > 0) {
		delete trackers.back();
		trackers.pop_back();
	}
}

/**
 * Returns the intrinsics matrix of this camera. This can either be the
 * result of a calibration, or a previously set intrinsics matrix.
 * The caller has to delete the result.
 *
 * @return A 3x3 matrix containing double values.
 */
cv::Mat* CvImageProcessor::getIntrinsicsMatrix()
{
	return new cv::Mat(*intrinsicsMatrix);
}

/**
 * Returns the distortion coefficients of this camera. This can either be
 * the result of a calibration, or previously set distortion coefficients.
 * The caller has to delete the result.
 *
 * @return A 5x1 matrix containing double values.
 */
cv::Mat* CvImageProcessor::getDistortionCoefficients()
{
	return new cv::Mat(*distortionCoefficients);
}

/**
 * Aborts the calibration. The calibration data it used before the
 * calibration will be kept.
 */
void CvImageProcessor::abortCalibration()
{
	abortCalibrationFlag = true;
}

/**
 * Returns true if the camera is calibrated. That means, the camera has a
 * not-null intrinsics matrix and a not-null distortion coefficients matrix.
 *
 * @return True if the camera is calibrated, false otherwise.
 */
bool CvImageProcessor::isCalibrated()
{
	return intrinsicsMatrix != 0 && distortionCoefficients != 0;
}

/**
 * Sets the receiver for the undistorted images.
 *
 * @param receiver The receiver.
 */
void CvImageProcessor::setUndistortedImageReceiver(IUndistortedImageReceiver *receiver)
{
	undistortedImageReceiver = receiver;
}

/**
 * Default destructor, ensures that all threads are stopped.
 */
CvImageProcessor::~CvImageProcessor()
{
	stopTracking();
}