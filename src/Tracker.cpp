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

// Use this to use the register keyword in some places. Might make things faster, but I didn't test it.
//#define QC_REGISTER

// Use this for debugging the object recognition. WARNING: Might lead to errors or strange behaviour when used with visualTracker == true.
//#define QC_DEBUG_TRACKER

Tracker::Tracker(ITrackerDataReceiver* dataReceiver, QuadcopterColor* color)
{
	this->dataReceiver = dataReceiver;
	this->thread = 0;
	this->image = 0;
	this->qc = color;
	
	this->showCameraImage = false;
	this->showMaskedImage = false;
}

Tracker::Tracker(ITrackerDataReceiver* dataReceiver, QuadcopterColor* color, bool showCameraImage, bool showMaskedImage)
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

Tracker::~Tracker()
{
}
	
void Tracker::start()
{
	if (thread != 0) {
		throw new std::runtime_error("Already started. (Maybe you forgot to call join?)");
	}
	
	if (showMaskedImage) {
		cv::namedWindow(maskedWindowName);
	}
	
	if (showCameraImage) {
		cv::namedWindow(cameraWindowName);
	}
	
	imageDirty = 0;
	stopFlag = false;
	
	thread = new boost::thread(boost::bind(&Tracker::executeTracker, this));
}

void Tracker::stop()
{
	if (thread == 0) {
		throw new std::runtime_error("Not started.");
	}
	
	stopFlag = true;
}

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

bool Tracker::isStarted()
{
	return thread != 0;
}
	
void Tracker::setNextImage(cv::Mat* image, long int time)
{
	imageMutex.lock();
	
	if (this->image != 0) {
		delete this->image;
	}
	
	this->image = new cv::Mat(*image);
	imageTime = time;
	imageDirty++;
	
	imageMutex.unlock();
}

QuadcopterColor* Tracker::getQuadcopterColor()
{
	return (QuadcopterColor*) qc;
}

void Tracker::drawCross(cv::Mat* mat, const int x, const int y)
{
	ROS_DEBUG("Drawing cross.");
	
	for (int i = x - 10; i <= x + 10; i++) {
		int j = y;
		
// 		ROS_DEBUG("i/j: %d/%d", i, j);
		if (i >= 0 && i < mat->rows && j >= 0 && j < mat->cols) {
			unsigned char* element = mat->data + mat->step[0] * j + mat->step[1] * i;
			
			unsigned char color;
			
			if (i + j % 3 > 0) {
				color = 0xFF;
			} else {
				color = 0;
			}
			
			element[0] = color;
			element[1] = color;
			element[2] = color;
		}
	}
	
	for (int j = y - 10; j <= y + 10; j++) {
		int i = x;
		
// 		ROS_DEBUG("i/j: %d/%d", i, j);
		if (i >= 0 && i < mat->rows && j >= 0 && j < mat->cols) {
			unsigned char* element = mat->data + mat->step[0] * j + mat->step[1] * i;
		
			unsigned char color;
			
			if (i + j % 3 > 0) {
				color = 0xFF;
			} else {
				color = 0;
			}
			
			element[0] = color;
			element[1] = color;
			element[2] = color;
		}
	}
	
	ROS_DEBUG("Cross drawed");
}

void Tracker::executeTracker()
{
	#ifdef QC_DEBUG_TRACKER
	cv::startWindowThread();
	cv::namedWindow("Tracker");
	#endif
	
	#define PI (3.1415926535897932384626433832795028841)
	// Kinect fow: 43° vertical, 57° horizontal
	double verticalScalingFactor = tan(43 * PI / 180) / 240;
	double horizontalScalingFactor = tan(57 * PI / 180) / 320;
	ROS_DEBUG("Scaling factors: %lf/%lf", horizontalScalingFactor, verticalScalingFactor);
	
	bool quadcopterTracked = false;
	
	while (!stopFlag) {
		if (imageDirty == 0) {
			usleep(100);
			continue;
		} else if (imageDirty > 1) {
			ROS_WARN("Skipped %d frames!", imageDirty - 1);
		}
		
		START_CLOCK(trackerClock)
		
		imageMutex.lock();
		cv::Mat* image = (cv::Mat*) this->image;
		image = new cv::Mat(*image);
		long int time = this->imageTime;
		imageDirty = 0;
		imageMutex.unlock();
		
		cv::Mat* cameraImage = 0;
		cv::Mat* maskedImage = 0;
		
		if (showCameraImage) {
			cameraImage = new cv::Mat(image->size(), image->type());
			image->copyTo(*cameraImage);
		}
		
		#ifdef QC_DEBUG_TRACKER
		cv::imshow("Tracker", *image);
		cv::waitKey(100000);
		#endif
		
		cv::Mat* mapImage = createColorMapImage(image);
		
		if (showMaskedImage) {
			// Convert to 3 channel image.
			maskedImage = new cv::Mat(cv::Size(640, 480), CV_8UC3);
			int target = 0;
		
			for (int i = 0; i < mapImage->total(); ++i) {
				maskedImage->data[target++] = mapImage->data[i];
				maskedImage->data[target++] = mapImage->data[i];
				maskedImage->data[target++] = mapImage->data[i];
			}
		}
		
		#ifdef QC_DEBUG_TRACKER
		cv::imshow("Tracker", *mapImage);
		cv::waitKey(100000);
		#endif
		
		cv::Mat morphKernel = cv::getStructuringElement(CV_SHAPE_RECT, cv::Size(5, 5));
		cv::morphologyEx(*mapImage, *mapImage, cv::MORPH_OPEN, morphKernel);
		
		// Finding blobs
		cvb::CvBlobs blobs;
		IplImage *labelImg = cvCreateImage(image->size(), IPL_DEPTH_LABEL, 1);
		IplImage iplMapImage = *mapImage;
		unsigned int result = cvLabel(&iplMapImage, labelImg, blobs);
		// ROS_DEBUG("Blob result: %d", result);
		
		// Filter blobs
		cvFilterByArea(blobs, 6, 1000000);
		
		#ifdef QC_DEBUG_TRACKER
		IplImage iplImage = *image;
		cvRenderBlobs(labelImg, blobs, &iplImage, &iplImage, CV_BLOB_RENDER_BOUNDING_BOX);
		cvb::CvTracks tracks;
		cvUpdateTracks(blobs, tracks, 200., 5);
		cvRenderTracks(tracks, &iplImage, &iplImage, CV_TRACK_RENDER_ID | CV_TRACK_RENDER_BOUNDING_BOX);
		cv::imshow("Tracker", cv::Mat(&iplImage));
		cv::waitKey(100000);
		cvReleaseTracks(tracks);
		ROS_DEBUG("Exiting debug block"); // TODO Tracking down issue #7
		#endif
		
		cvb::CvTracks tracks;
		
		if (showCameraImage || showMaskedImage) {
			cvUpdateTracks(blobs, tracks, 200., 5);
		}
		
		if (showMaskedImage) {
			IplImage iplImage = *maskedImage;
			cvRenderBlobs(labelImg, blobs, &iplImage, &iplImage, CV_BLOB_RENDER_BOUNDING_BOX);
			cvRenderTracks(tracks, &iplImage, &iplImage, CV_TRACK_RENDER_ID | CV_TRACK_RENDER_BOUNDING_BOX);
			ROS_DEBUG("Exiting visual masked block"); // TODO Tracking down issue #7
		}
		
		if (showCameraImage) {
			IplImage iplImage = *cameraImage;
			cvRenderBlobs(labelImg, blobs, &iplImage, &iplImage, CV_BLOB_RENDER_BOUNDING_BOX);
			cvRenderTracks(tracks, &iplImage, &iplImage, CV_TRACK_RENDER_ID | CV_TRACK_RENDER_BOUNDING_BOX);
			ROS_DEBUG("Exiting visual masked block"); // TODO Tracking down issue #7
		}
		
		if (showCameraImage || showMaskedImage) {
			cvReleaseTracks(tracks);
		}
		
		if (blobs.size() != 0) {
			// Find biggest blob
			cvb::CvLabel largestBlob =  cvLargestBlob(blobs);
			CvPoint2D64f center = blobs.find(largestBlob)->second->centroid;
			double x = center.x;
			double y = center.y;
			
			// Set (0, 0) to center.
			x -= 320;
			y -= 240;
			//ROS_DEBUG("Center: %lf/%lf", x, y);
			
			// Apply scaling
			x *= horizontalScalingFactor;
			y *= verticalScalingFactor;
			
			dataReceiver->receiveTrackingData(cv::Scalar(x, y, 1.0), ((QuadcopterColor*) qc)->getId(), time);
			
			if (showMaskedImage) {
				drawCross(maskedImage, center.x, center.y);
			}
			
			if (showCameraImage) {
				drawCross(cameraImage, center.x, center.y);
			}
			
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
		cvReleaseImage(&labelImg);
		
		// ROS_DEBUG("cvb stuff freed"); // TODO Tracking down issue #7
		
		if (showMaskedImage) {
			cv::imshow(maskedWindowName, *maskedImage);
			delete maskedImage;
			
			ROS_DEBUG("showed masked image"); // TODO Tracking down issue #7
		}
		
		if (showCameraImage) {
			cv::imshow(cameraWindowName, *cameraImage);
			delete cameraImage;
			
			ROS_DEBUG("showed camera image"); // TODO Tracking down issue #7
		}
		
		delete mapImage;
		delete image;
		
		STOP_CLOCK(trackerClock, "Calculation of quadcopter position took: ")
	}
	
	#ifdef QC_DEBUG_TRACKER
	cv::destroyWindow("Tracker");
	#endif
	
	if (showMaskedImage) {
		cv::destroyWindow(maskedWindowName);
	}
	
	if (showCameraImage) {
		cv::destroyWindow(cameraWindowName);
	}
	
	ROS_INFO("Tracker with id %d terminated", ((QuadcopterColor*) this->qc)->getId());
}

cv::Mat* Tracker::createColorMapImage(cv::Mat* image) {
	START_CLOCK(convertColorClock)
	
	cv::Mat* mapImage = new cv::Mat(image->size(), CV_8UC1);
	
	// Debug HSV color range
	// unsigned char* element = image->data + image->step[0] * 240 + image->step[1] * 320;
	// ROS_DEBUG("R: %d G: %d B: %d", element[2], element[1], element[0]);
	
	cv::cvtColor(*image, *image, CV_BGR2HSV);
	
	// Debug HSV color range
	// ROS_DEBUG("H: %d S: %d V: %d", element[0], element[1], element[2]);
	
	STOP_CLOCK(convertColorClock, "Converting colors took: ")
	START_CLOCK(maskImageClock)
	
	// Trying to make this fast.
	#ifdef QC_REGISTER
	register uint8_t *current, *end, *source;
	register int minHue, maxHue, minSaturation, /*maxSaturation,*/ minValue/*, maxValue*/;
	#else
	uint8_t *current, *end, *source;
	int minHue, maxHue, minSaturation, /*maxSaturation,*/ minValue/*, maxValue*/;
	#endif
	
	QuadcopterColor* color = (QuadcopterColor*) qc;
	
	minHue = color->getMinColor().val[0];
	maxHue = color->getMaxColor().val[0];
	minSaturation = color->getMinColor().val[1];
	//maxSaturation = color->getMaxColor().val[1]; // unused
	minValue = color->getMinColor().val[2];
	//maxValue = color->getMaxColor().val[2]; // unused
	
	end = mapImage->data + mapImage->size().width * mapImage->size().height;
	source = image->data;
	
	if (minHue < maxHue) {
		for (current = mapImage->data; current < end; ++current, source += 3) {
			//if (*source > maxHue || *source < minHue || *(++source) > maxSaturation || *source < minSaturation || *(++source) > maxValue || *(source++) < minValue) {
			if (*source > maxHue || *source < minHue || *(source + 1) < minSaturation || *(source + 2) < minValue) {
				*current = 0;
			} else {
				*current = 255;
			}
		}
	} else {
		// Hue interval inverted here.
		for (current = mapImage->data; current < end; ++current, source += 3) {
			//if (*source < maxHue || *source > minHue || *(++source) > maxSaturation || *source < minSaturation || *(++source) > maxValue || *(source++) < minValue) {
			if ((*source > maxHue && *source < minHue) || *(source + 1) < minSaturation || *(source + 2) < minValue) {
				*current = 0;
			} else {
				*current = 255;
			}
		}
	}
	
	STOP_CLOCK(maskImageClock, "Image masking took: ")
	
	return mapImage;
}
