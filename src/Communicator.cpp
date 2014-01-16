
#include "Communicator.hpp"

Communicator::Communicator()
{
	ros::NodeHandle n;

	this->initializeCameraService = n.advertiseService("initialize_camera", &Communicator::handleInitializeCameraService, this);
}

bool Communicator::handleInitializeCameraService(
		camera_application::InitializeCameraService::Request &req,
		camera_application::InitializeCameraService::Response &res)
{
	ROS_INFO("Received initialize_camera message.");
	res.error = 0;
	return true;
}
