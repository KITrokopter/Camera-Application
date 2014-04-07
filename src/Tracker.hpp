#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <boost/thread.hpp>
#include <opencv2/core/core.hpp>
#include <string>

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
	volatile long int imageTime;
	volatile int imageDirty;
	Mutex imageMutex;
	ITrackerDataReceiver* dataReceiver;
	
	bool showCameraImage;
	bool showMaskedImage;
	std::string maskedWindowName;
	std::string cameraWindowName;
	
	// Methods
	void executeTracker();
	cv::Mat createColorMapImage(cv::Mat& image, cv::Mat& mapImage, cv::Mat& hsvImage);
	void drawCross(cv::Mat mat, int i, int j);
	
public:
	Tracker(ITrackerDataReceiver* dataReceiver, QuadcopterColor* color);
	Tracker(ITrackerDataReceiver* dataReceiver, QuadcopterColor* color, bool showCameraImage, bool showMaskedImage);
	~Tracker();
	
	void start();
	void stop();
	void join();
	bool isStarted();
	
	/**
	 * Creates a copy of the given image and uses it for tracking as soon as it can.
	 * The given image must already be undistorted.
	 */
	void setNextImage(cv::Mat* image, long int time);
	
	QuadcopterColor* getQuadcopterColor();
};

#endif // TRACKER_HPP
