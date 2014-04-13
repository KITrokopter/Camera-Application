#ifndef ITRACKER_DATA_RECEIVER_HPP
#define ITRACKER_DATA_RECEIVER_HPP

/**
 * Interface for receiving tracking data.
 *
 * @author Sebastian Schmidt
 */
class ITrackerDataReceiver {
public:
	/**
	 * Passes the result of the analysation of one image and one color.
	 *
	 * @param direction The 3D-vector pointing into the direction of the
	 *quadcopter, as seen from the camera.
	 *                  The (0, 0, 1) vector is the middle of the image,
	 *negative x is left, negative y down.
	 * @param id The id of the quadcopter this vector belongs to.
	 * @param time The time the image that was used to calculate this vector was
	 *taken.
	 */
	virtual void receiveTrackingData(cv::Scalar direction, int id, long int time) = 0;
};

#endif // ITRACKER_DATA_RECEIVER_HPP
