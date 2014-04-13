#include "Tracker.hpp"

#include <stdexcept>
#include <iostream>
#include <stdlib.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cvblob.h>

#include <ros/console.h>
#include <cmath>

#include "profiling.hpp"


/**
 * Creates a new tracker for the given quadcopter.
 * showCameraImage and showMaskedImage default to false.
 *
 * @param dataReceiver The receiver for the tracking data.
 * @param color The quadcopter id <-> color mapping.
 */
Tracker::Tracker(ITrackerDataReceiver *dataReceiver, QuadcopterColor *color)
{
	this->dataReceiver = dataReceiver;
	this->thread = 0;
	this->image = 0;
	this->qc = color;

	this->showCameraImage = false;
	this->showMaskedImage = false;
}

/**
 * Creates a new tracker for the given quadcopter.
 *
 * @param dataReceiver The receiver for the tracking data.
 * @param color The quadcopter id <-> color mapping.
 * @param showCameraImage True if the camera image should be shown during tracking.
 * (Resource intensive and needs $DISPLAY to be set.)
 * @param showMaskedImage True if the color masked image should be shown during tracking.
 * (Resource intensive and needs $DISPLAY to be set.)
 */
Tracker::Tracker(ITrackerDataReceiver *dataReceiver, QuadcopterColor *color, bool showCameraImage, bool showMaskedImage)
{
	this->dataReceiver = dataReceiver;
	this->thread = 0;
	this->image = 0;
	this->qc = color;

	this->showCameraImage = showCameraImage;
	this->showMaskedImage = showMaskedImage;

	if (showMaskedImage) {
		std::stringstream ss;
		ss << "Tracker of id " << ((QuadcopterColor*) this->qc)->getId() << " showing masked image";

		this->maskedWindowName = ss.str();
		cv::startWindowThread();
	}

	if (showCameraImage) {
		std::stringstream ss;
		ss << "Tracker of id " << ((QuadcopterColor*) this->qc)->getId() << " showing camera image";

		this->cameraWindowName = ss.str();
		cv::startWindowThread();
	}
}

/**
 * Destroys the tracker and stops the tracking thread, if it is running.
 */
Tracker::~Tracker()
{
	if (isStarted()) {
		stop();
		join();
	}
}

/**
 * Starts the tracking using a new thread.
 */
void Tracker::start()
{
	if (thread != 0)
		throw new std::runtime_error("Already started. (Maybe you forgot to call join?)");

	if (showMaskedImage)
		cv::namedWindow(maskedWindowName);

	if (showCameraImage)
		cv::namedWindow(cameraWindowName);

	imageDirty = 0;
	stopFlag = false;

	thread = new boost::thread(boost::bind(&Tracker::executeTracker, this));
}

/**
 * Requests the tracking thread to stop.
 */
void Tracker::stop()
{
	if (thread == 0)
		throw new std::runtime_error("Not started.");

	stopFlag = true;
}

/**
 * Joins the tracking thread.
 */
void Tracker::join()
{
	if (thread != 0) {
		thread->join();
		delete thread;
		thread = 0;
	}

	if (this->image != 0) {
		delete this->image;
		this->image = 0;
	}
}

/**
 * Returns true, if the tracking is started.
 *
 * @return True, if the tracking is started.
 */
bool Tracker::isStarted()
{
	return thread != 0;
}

/**
 * Sets the next image for the tracker. The old image is deleted.
 *
 * @param image The image.
 * @param time The time the image was taken.
 */
void Tracker::setNextImage(cv::Mat *image, long int time)
{
	imageMutex.lock();

	if (this->image != 0)
		delete this->image;

	this->image = new cv::Mat(*image);
	imageTime = time;
	imageDirty++;

	imageMutex.unlock();
}

/**
 * Returns the QuadcopterColor that is tracked by this object.
 *
 * @return The QuadcopterColor that is tracked by this object.
 */
QuadcopterColor* Tracker::getQuadcopterColor()
{
	return (QuadcopterColor*) qc;
}

/**
 * Draws a cross to the specified location on the given image.
 *
 * @param mat The image.
 * @param x The x coordinate of the cross center.
 * @param y The y coordinage of the cross center.
 */
void Tracker::drawCross(cv::Mat &mat, const int x, const int y)
{
	for (int i = x - 10; i <= x + 10; i++) {
		int j = y;

//      ROS_DEBUG("i/j: %d/%d", i, j);
		if (i >= 0 && i < mat.rows && j >= 0 && j < mat.cols) {
			unsigned char *element = mat.data + mat.step[0] * j + mat.step[1] * i;

			unsigned char color;

			if ((i + j) % 3 != 0)
				color = 0xFF;
			else
				color = 0;

			element[0] = color;
			element[1] = color;
			element[2] = color;
		}
	}

	for (int j = y - 10; j <= y + 10; j++) {
		int i = x;

//      ROS_DEBUG("i/j: %d/%d", i, j);
		if (i >= 0 && i < mat.rows && j >= 0 && j < mat.cols) {
			unsigned char *element = mat.data + mat.step[0] * j + mat.step[1] * i;

			unsigned char color;

			if ((i + j) % 3 != 0)
				color = 0xFF;
			else
				color = 0;

			element[0] = color;
			element[1] = color;
			element[2] = color;
		}
	}
}

/**
 * Runs the tracking.
 */
void Tracker::executeTracker()
{
	#define PI (3.1415926535897932384626433832795028841)
	// Kinect fow: 43° vertical, 57° horizontal
	double verticalScalingFactor = tan(43 * PI / 180) / 240;
	double horizontalScalingFactor = tan(57 * PI / 180) / 320;
	ROS_DEBUG("Scaling factors: %lf/%lf", horizontalScalingFactor, verticalScalingFactor);

	bool quadcopterTracked = false;

	// Images
	cv::Mat cameraImage(cv::Size(640, 480), CV_8UC3); // Only for showCameraImage == true.
	cv::Mat maskedImage(cv::Size(640, 480), CV_8UC3); // Only for showMaskedImage == true.
	cv::Mat image(cv::Size(640, 480), CV_8UC3); // The raw image from the camera.
	cv::Mat mapImage(cv::Size(640, 480), CV_8UC1); // The color mapped image.
	cv::Mat hsvImage(cv::Size(640, 480), CV_8UC3);  // The raw image in hsv format.

	// CvBlob
	cvb::CvBlobs blobs;
	IplImage *labelImg = cvCreateImage(image.size(), IPL_DEPTH_LABEL, 1);
	cv::Mat morphKernel = cv::getStructuringElement(CV_SHAPE_RECT, cv::Size(5, 5));
	cvb::CvTracks tracks;
	IplImage iplMapImage;

	while (!stopFlag) {
		if (imageDirty == 0) {
			usleep(100);
			continue;
		} else if (imageDirty > 1) {
			ROS_WARN("Skipped %d frames!", imageDirty - 1);
		}

		START_CLOCK(trackerClock)

		imageMutex.lock();
		((cv::Mat*) this->image)->copyTo(image);
		long int time = this->imageTime;
		imageDirty = 0;
		imageMutex.unlock();

		if (showCameraImage)
			image.copyTo(cameraImage);

		createColorMapImage(image, mapImage, hsvImage);

		if (showMaskedImage) {
			// Convert to 3 channel image.
			int target = 0;

			for (int i = 0; i < mapImage.total(); ++i) {
				maskedImage.data[target++] = mapImage.data[i];
				maskedImage.data[target++] = mapImage.data[i];
				maskedImage.data[target++] = mapImage.data[i];
			}
		}

		cv::morphologyEx(mapImage, mapImage, cv::MORPH_OPEN, morphKernel);

		// Finding blobs
		// Only copies headers.
		iplMapImage = mapImage;
		unsigned int result = cvLabel(&iplMapImage, labelImg, blobs);
		// ROS_DEBUG("Blob result: %d", result);

		// Filter blobs
		cvFilterByArea(blobs, 10, 1000000);

		if (showCameraImage || showMaskedImage)
			cvUpdateTracks(blobs, tracks, 200., 5);

		if (showMaskedImage) {
			// Only copies headers.
			IplImage iplImage = maskedImage;
			cvRenderBlobs(labelImg, blobs, &iplImage, &iplImage, CV_BLOB_RENDER_BOUNDING_BOX);
			cvRenderTracks(tracks, &iplImage, &iplImage, CV_TRACK_RENDER_ID | CV_TRACK_RENDER_BOUNDING_BOX);
			ROS_DEBUG("Exiting visual masked block"); // TODO Tracking down
			                                          // issue #7
		}

		if (showCameraImage) {
			// Only copies headers.
			IplImage iplImage = cameraImage;
			cvRenderBlobs(labelImg, blobs, &iplImage, &iplImage, CV_BLOB_RENDER_BOUNDING_BOX);
			cvRenderTracks(tracks, &iplImage, &iplImage, CV_TRACK_RENDER_ID | CV_TRACK_RENDER_BOUNDING_BOX);
			ROS_DEBUG("Exiting visual masked block"); // TODO Tracking down
			                                          // issue #7
		}

		if (showCameraImage || showMaskedImage)
			cvReleaseTracks(tracks);

		if (blobs.size() != 0) {
			// Find biggest blob
			cvb::CvLabel largestBlob = cvLargestBlob(blobs);
			CvPoint2D64f center = blobs.find(largestBlob)->second->centroid;
			double x = center.x;
			double y = center.y;

			// Set (0, 0) to center.
			x -= 320;
			y -= 240;
			// ROS_DEBUG("Center: %lf/%lf", x, y);

			// Apply scaling
			x *= horizontalScalingFactor;
			y *= verticalScalingFactor;

			dataReceiver->receiveTrackingData(cv::Scalar(x, y, 1.0), ((QuadcopterColor*) qc)->getId(), time);

			if (showMaskedImage)
				drawCross(maskedImage, center.x, center.y);

			if (showCameraImage)
				drawCross(cameraImage, center.x, center.y);

			if (!quadcopterTracked) {
				quadcopterTracked = true;
				ROS_DEBUG("Quadcopter %d tracked", ((QuadcopterColor*) this->qc)->getId());
			}
		} else if (quadcopterTracked) {
			quadcopterTracked = false;
			ROS_DEBUG("Quadcopter %d NOT tracked", ((QuadcopterColor*) this->qc)->getId());
		}

		// Free cvb stuff.
		cvReleaseBlobs(blobs);

		// ROS_DEBUG("cvb stuff freed"); // TODO Tracking down issue #7

		if (showMaskedImage) {
			cv::imshow(maskedWindowName, maskedImage);

			ROS_DEBUG("showed masked image"); // TODO Tracking down issue #7
		}

		if (showCameraImage) {
			cv::imshow(cameraWindowName, cameraImage);

			ROS_DEBUG("showed camera image"); // TODO Tracking down issue #7
		}

		STOP_CLOCK(trackerClock, "Calculation of quadcopter position took: ")
	}

	cvReleaseImage(&labelImg);

	if (showMaskedImage)
		cv::destroyWindow(maskedWindowName);

	if (showCameraImage)
		cv::destroyWindow(cameraWindowName);

	ROS_INFO("Tracker with id %d terminated", ((QuadcopterColor*) this->qc)->getId());
}

/**
 * Filters the given image using the QuadcopterColor.
 * The result is written to mapImage. White pixels mean in range,
 * black pixels mean out of range.
 *
 * @param image The raw image.
 * @param mapImage The resulting mapped image. (Output array)
 * @param hsvImage The raw image in hsv format. (Output array)
 */
cv::Mat Tracker::createColorMapImage(cv::Mat &image, cv::Mat &mapImage, cv::Mat &hsvImage)
{
	START_CLOCK(convertColorClock)

	// This ensures that the mapImage has a buffer.
	// Since the buffer is never freed during tracking, this only allocates memory once.
	mapImage.reserve(480);

	cv::cvtColor(image, hsvImage, CV_BGR2HSV);

	STOP_CLOCK(convertColorClock, "Converting colors took: ")
	START_CLOCK(maskImageClock)

	uint8_t * current, *end, *source;
	int minHue, maxHue, minSaturation, minValue;

	QuadcopterColor *color = (QuadcopterColor*) qc;

	minHue = color->getMinColor().val[0];
	maxHue = color->getMaxColor().val[0];
	minSaturation = color->getMinColor().val[1];
	minValue = color->getMinColor().val[2];

	end = mapImage.data + mapImage.size().width * mapImage.size().height;
	source = hsvImage.data;

	if (minHue < maxHue)
		for (current = mapImage.data; current < end; ++current, source += 3) {
			if (*source > maxHue || *source < minHue || *(source + 1) < minSaturation || *(source + 2) < minValue)
				*current = 0;
			else
				*current = 255;
		}
	else
		// Hue interval inverted here.
		for (current = mapImage.data; current < end; ++current, source += 3) {
			if ((*source > maxHue && *source < minHue) || *(source + 1) < minSaturation || *(source + 2) < minValue)
				*current = 0;
			else
				*current = 255;
		}

	STOP_CLOCK(maskImageClock, "Image masking took: ")

	return mapImage;
}