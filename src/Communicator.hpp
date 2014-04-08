#pragma once

#include <vector>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>

#include "CvImageProcessor.hpp"
#include "CvKinect.hpp"

// Services
#include "camera_application/InitializeCameraService.h"

// Subscribers
#include "camera_application/PictureSendingActivation.h"
#include "camera_application/CalibrateCamera.h"
#include "api_application/System.h"

// Publishers
#include "camera_application/Picture.h"
#include "camera_application/RawPosition.h"
#include "camera_application/CameraCalibrationData.h" // Also subscriber


/*
 * Class: Communicator
 *
 * Provides communication with ROS.
 */
class Communicator : public IImageReceiver, public ITrackerDataReceiver,
	public ICalibrationFinishedListener, public IUndistortedImageReceiver {
public:
	Communicator(CvKinect *device, CvImageProcessor *analyzer);

	void receiveImage(cv::Mat *image, long int time, int type);
	void receiveTrackingData(cv::Scalar direction, int id, long int time);
	void calibrationFinished(cv::Mat *intrinsicsMatrix, cv::Mat *distortionCoefficients);
	void receiveUndistortedImage(cv::Mat *image, long int time);

	/**
	 * Sends a picture via ROS using the Picture topic.
	 *
	 * @param data The image data.
	 * @param timestamp The time the image was taken.
	 * @param type The type of the image (0 for normal image, 1 for calibration
	 *image).
	 */
	void sendPicture(camera_application::Picture::_image_type &data, uint64_t timestamp, int type);

protected:
	// Services
	bool handleInitializeCameraService(
	    camera_application::InitializeCameraService::Request &req,
	    camera_application::InitializeCameraService::Response &res);

	// Subscribers
	void handlePictureSendingActivation(
	    const camera_application::PictureSendingActivation::Ptr &msg);

	void handleCalibrateCamera(
	    const camera_application::CalibrateCamera::Ptr &msg);

	void handleCameraCalibrationData(
	    const camera_application::CameraCalibrationData::Ptr &msg);

	void handleSystem(
	    const api_application::System::Ptr &msg);

private:
	ros::ServiceServer initializeCameraService;
	ros::Subscriber pictureSendingActivationSubscriber;
	ros::Subscriber calibrateCameraSubscriber;
	ros::Subscriber cameraCalibrationDataSubscriber;
	ros::Subscriber systemSubscriber;
	ros::Publisher picturePublisher;
	ros::Publisher rawPositionPublisher;
	ros::Publisher cameraCalibrationDataPublisher;

	// Tracking
	CvKinect *device;
	CvImageProcessor *analyzer;

	// Initialization data
	bool initialized;
	bool pictureSendingActivated;
	bool firstPictureReceived;
	uint32_t id;
	uint32_t pictureNumber;
	std::vector<uint32_t> hsvColorRanges;
	std::vector<uint32_t> quadCopterIds;
};
