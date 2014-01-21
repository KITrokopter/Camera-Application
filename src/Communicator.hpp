#pragma once

#include "ros/ros.h"

#include "ImageAnalyzer.hpp"

// Services
#include "camera_application/InitializeCameraService.h"

// Subscribers
#include "camera_application/PictureSendingActivation.h"

/*
 * Class: Communicator
 *
 * Provides communication with ROS.
 */
class Communicator {
	public:
		Communicator(ImageAnalyzer *analyzer);

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
		ImageAnalyzer *analyzer;
};
