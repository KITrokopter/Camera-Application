#ifndef ITRACKER_DATA_RECEIVER_HPP
#define ITRACKER_DATA_RECEIVER_HPP

class ITrackerDataReceiver
{
public:
	virtual void receiveTrackingData(cv::Scalar direction, int id);
};

#endif // ITRACKER_DATA_RECEIVER_HPP