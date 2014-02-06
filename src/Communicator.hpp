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

// Publishers
#include "camera_application/Picture.h"
#include "camera_application/RawPosition.h"


/*
 * Class: Communicator
 *
 * Provides communication with ROS.
 */
class Communicator : public IImageReceiver, public ITrackerDataReceiver {
	public:
		Communicator(CvKinect *device, CvImageProcessor *analyzer);

		void receiveImage(cv::Mat* image, long int time);
		void receiveTrackingData(cv::Scalar direction, int id, long int time);
		void sendPicture(camera_application::Picture::_image_type &data, uint64_t timestamp);

	protected:
		// Services
		bool handleInitializeCameraService(
				camera_application::InitializeCameraService::Request &req,
				camera_application::InitializeCameraService::Response &res);

		// Subscribers
		void handlePictureSendingActivation(
				const camera_application::PictureSendingActivation::Ptr &msg);

	private:
		ros::ServiceServer initializeCameraService;
		ros::Subscriber pictureSendingActivationSubscriber;
		ros::Publisher picturePublisher;
		ros::Publisher rawPositionPublisher;

		CvKinect *device;
		CvImageProcessor *analyzer;

		// Initialization data
		bool initialized;
		uint32_t id;
		std::vector<uint32_t> hsvColorRanges;
		std::vector<uint32_t> quadCopterIds;
};
