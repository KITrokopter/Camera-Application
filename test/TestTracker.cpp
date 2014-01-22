#include "../src/Tracker.hpp"
#include "../src/ITrackerDataReceiver.hpp"
#include <iostream>
#include <opencv2/core/core.hpp>

#include <ctime>    // For time()
#include <cstdlib>  // For srand() and rand()

class TestTracker : public ITrackerDataReceiver
{
public:
	TestTracker(){}
	~TestTracker(){}
	
	void receiveTrackingData(cv::Scalar direction, int id)
	{
		std::cout << "Id: " << id << std::endl;
		std::cout << "Direction: " << direction.val[0] << ", " << direction.val[1] << ", " << direction.val[2] << std::endl;
	}
};

int main(void)
{
	srand(time(0));
	
	QuadcopterColor* color = new QuadcopterColor(0, 20, 30, 255, 30, 220, 0);
	Tracker tracker(new TestTracker(), color);
	
	tracker.start();
	std::cout << "Tracker started" << std::endl;
	
	cv::Mat* mat = new cv::Mat(cv::Size(640, 480), CV_8UC3);
	for (int i = 0; i < 640 * 480 * 3; i++) {
		mat->data[i] = rand() % 256;
	}
	
	tracker.setNextImage(mat);
	
	// Give tracker time to detect image before stopping.
	usleep(1000000);
	
	tracker.stop();
	std::cout << "Tracker stopped" << std::endl;
	
	tracker.join();
	std::cout << "Tracker joined" << std::endl;
}