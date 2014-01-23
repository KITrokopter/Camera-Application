#include "Tracker.hpp"

#include <stdexcept>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/console.h>
#include <cvblob.h>
#include <cmath>
#include "profiling.hpp"

// Use this to use the register keyword in some places. Might make things faster, but I didn't test it.
//#define QC_REGISTER

// Use this for debugging the object recognition.
#define QC_DEBUG_TRACKER

#ifdef QC_DEBUG_TRACKER
#include <opencv2/highgui/highgui.hpp>
#endif

Tracker::Tracker(ITrackerDataReceiver* dataReceiver, QuadcopterColor* color)
{
	this->dataReceiver = dataReceiver;
	this->thread = 0;
	this->image = 0;
	this->qc = color;
}
	
void Tracker::start()
{
	if (thread != 0) {
		throw new std::runtime_error("Already started. (Maybe you forgot to call join?)");
	}
	
	imageDirty = false;
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
	
void Tracker::setNextImage(cv::Mat* image, long int time)
{
	imageMutex.lock();
	
	if (this->image != 0) {
		delete this->image;
	}
	
	this->image = new cv::Mat(*image);
	imageTime = time;
	imageDirty = true;
	
	imageMutex.unlock();
}

QuadcopterColor* Tracker::getQuadcopterColor()
{
	return (QuadcopterColor*) qc;
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
	
	while (!stopFlag) {
		if (!imageDirty) {
			usleep(100);
			continue;
		}
		
		START_CLOCK(trackerClock)
		
		imageMutex.lock();
		cv::Mat* image = (cv::Mat*) this->image;
		image = new cv::Mat(*image);
		long int time = this->imageTime;
		imageMutex.unlock();
		
		#ifdef QC_DEBUG_TRACKER
		cv::imshow("Tracker", *image);
		cv::waitKey(100000);
		#endif
		
		cv::Mat* mapImage = createColorMapImage(image);
		
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
		ROS_DEBUG("Blob result: %d", result);
		
		// Filter blobs
		cvFilterByArea(blobs, 10, 1000000);
		
		#ifdef QC_DEBUG_TRACKER
		IplImage iplImage = *image;
		cvRenderBlobs(labelImg, blobs, &iplImage, &iplImage, CV_BLOB_RENDER_BOUNDING_BOX);
		cvb::CvTracks tracks;
		cvUpdateTracks(blobs, tracks, 200., 5);
		cvRenderTracks(tracks, &iplImage, &iplImage, CV_TRACK_RENDER_ID | CV_TRACK_RENDER_BOUNDING_BOX);
		cv::imshow("Tracker", cv::Mat(&iplImage));
		cv::waitKey(100000);
		cvReleaseTracks(tracks);
		#endif
		
		// Find biggest blob
		cvb::CvLabel largestBlob =  cvLargestBlob(blobs);
		CvPoint2D64f center = blobs.find(largestBlob)->second->centroid;
		double x = center.x;
		double y = center.y;
		
		// Set (0, 0) to center.
		x -= 320;
		y = 240 - y;
		ROS_DEBUG("Center: %lf/%lf", x, y);
		
		// Apply scaling
		x *= horizontalScalingFactor;
		y *= verticalScalingFactor;
		
		dataReceiver->receiveTrackingData(cv::Scalar(x, y, 1.0), ((QuadcopterColor*) qc)->getId(), time);
		
		// Free cvb stuff. TODO probably incomplete.
		cvReleaseBlobs(blobs);
		
		delete mapImage;
		delete image;
		
		STOP_CLOCK(trackerClock, "Calculation of quadcopter position took: ")
	}
	
	#ifdef QC_DEBUG_TRACKER
	cv::destroyWindow("Tracker");
	#endif
}

cv::Mat* Tracker::createColorMapImage(cv::Mat* image) {
	START_CLOCK(convertColorClock)
	
	cv::Mat* mapImage = new cv::Mat(image->size(), CV_8UC1);
	cv::cvtColor(*image, *image, CV_RGB2HSV);
	
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
			if (*source > maxHue || *(source) < minHue || *(source + 1) < minSaturation || *(source + 2) < minValue) {
				*current = 0;
			} else {
				*current = 255;
			}
		}
	} else {
		// Hue interval inverted here.
		for (current = mapImage->data; current < end; ++current, source += 3) {
			//if (*source < maxHue || *source > minHue || *(++source) > maxSaturation || *source < minSaturation || *(++source) > maxValue || *(source++) < minValue) {
			if (*source < maxHue || *(source) > minHue || *(source + 1) < minSaturation || *(source + 2) < minValue) {
				*current = 0;
			} else {
				*current = 255;
			}
		}
	}
	
	STOP_CLOCK(maskImageClock, "Image masking took: ")
	
	return mapImage;
}
