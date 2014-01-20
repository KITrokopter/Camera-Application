#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <boost/thread.hpp>
#include <opencv2/core/core.hpp>

#include "QuadcopterColor.hpp"
#include "Mutex.hpp"
#include "ITrackerDataReceiver.hpp"

class Tracker
{
private:
	// Thread control
	boost::thread* thread;
	volatile bool stopFlag;
	
	// Tracking
	volatile QuadcopterColor* qc;
	volatile cv::Mat* image;
	volatile bool imageDirty;
	Mutex imageMutex;
	ITrackerDataReceiver* dataReceiver;
	
	// Methods
	void executeTracker();
	cv::Mat* createColorMapImage(cv::Mat* image);
	
public:
	Tracker(ITrackerDataReceiver* dataReceiver, QuadcopterColor* color);
	
	void start();
	void stop();
	void join();
	
	/*
	 * Method: setNextImage
	 * 
	 * Creates a copy of the given image and uses it for tracking as soon as it can.
	 * The given image must already be undistorted.
	 */
	void setNextImage(cv::Mat* image);
	
	QuadcopterColor* getQuadcopterColor();
};

#endif // TRACKER_HPP