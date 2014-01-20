#include "Tracker.hpp"

#include <stdexcept>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/chrono.hpp>

// Use this to use the register keyword in some places. Might make things faster, but I didn't test it.
//#define QC_REGISTER

// Use this to output profiling information.
#define QC_PROFILE

#ifdef QC_PROFILE
#define START_CLOCK(clock) boost::chrono::high_resolution_clock::time_point clock = boost::chrono::high_resolution_clock::now();
#define STOP_CLOCK(clock, message) std::cout << message << boost::chrono::duration_cast<boost::chrono::nanoseconds>(boost::chrono::high_resolution_clock::now() - clock) << std::endl;
#else
#define START_CLOCK(clock) ;
#define STOP_CLOCK(clock, message) ;
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
	while (!stopFlag) {
		if (!imageDirty) {
			usleep(100);
			continue;
		}
		
		START_CLOCK(trackerClock)
		
		imageMutex.lock();
		cv::Mat* image = (cv::Mat*) this->image;
		imageMutex.unlock();
		
		cv::Mat* mapImage = createColorMapImage(image);
		
		STOP_CLOCK(trackerClock, "Calculation of quadcopter position took: ")
	}
}

cv::Mat* Tracker::createColorMapImage(cv::Mat* image) {
	START_CLOCK(convertColorClock)
	
	cv::Mat* mapImage = new cv::Mat(image->size(), CV_8UC1);
	cv::cvtColor(*image, *image, CV_RGB2HSV);
	
	STOP_CLOCK(convertColorClock, "Converting colors took: ")
	START_CLOCK(maskImageClock)
	
	// Trying to make this fast.
#ifdef QC_REGISTER
	register char* current, end, source;
	register int minHue, maxHue, minSaturation, maxSaturation, minValue, maxValue;
#else
	char* current, end, source;
	int minHue, maxHue, minSaturation, maxSaturation, minValue, maxValue;
#endif
	
	end = mapImage->data + mapImage->size().width * mapImage->size().height;
	source = image->data;
	
	for (current = mapImage->data; current < end; current++, soure += 3) {
		
	}
	
	STOP_CLOCK(maskImageClock, "Image masking took: ")
}
