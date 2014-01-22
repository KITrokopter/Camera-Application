#include "Tracker.hpp"

#include <stdexcept>
#include <opencv2/imgproc/imgproc.hpp>
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
	
void Tracker::setNextImage(cv::Mat* image)
{
	imageMutex.lock();
	
	if (this->image != 0) {
		delete this->image;
	}
	
	this->image = new cv::Mat(*image);
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
	
	while (!stopFlag) {
		if (!imageDirty) {
			usleep(100);
			continue;
		}
		
		START_CLOCK(trackerClock)
		
		imageMutex.lock();
		cv::Mat* image = (cv::Mat*) this->image;
		image = new cv::Mat(*image);
		imageMutex.unlock();
		
		#ifdef QC_DEBUG_TRACKER
		cv::imshow("Tracker", *image);
		cv::waitKey(1000);
		#endif
		
		cv::Mat* mapImage = createColorMapImage(image);
		
		#ifdef QC_DEBUG_TRACKER
		cv::imshow("Tracker", *mapImage);
		cv::waitKey(1000);
		#endif
		
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
	
	for (current = mapImage->data; current < end; ++current) {
		//if (*source > maxHue || *source < minHue || *(++source) > maxSaturation || *source < minSaturation || *(++source) > maxValue || *(source++) < minValue) {
		if (*source > maxHue || *source < minHue || *(++source) < minSaturation || *(++source) < minValue) {
			*current = 0;
		} else {
			*current = 255;
		}
		
		++source;
	}
	
	STOP_CLOCK(maskImageClock, "Image masking took: ")
	
	return mapImage;
}
