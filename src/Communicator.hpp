#pragma once

#include "ros/ros.h"

// Services
#include "camera_application/InitializeCameraService.h"

/*
 * Class: Communicator
 *
 * Provides communication with ROS.
 */
class Communicator {
	public:
		Communicator();

	protected:
		bool handleInitializeCameraService(
				camera_application::InitializeCameraService::Request &req,
				camera_application::InitializeCameraService::Response &res);
	private:
		ros::ServiceServer initializeCameraService;
};
