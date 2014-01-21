
#include "Communicator.hpp"

#include <ctime>

Communicator::Communicator(ImageAnalyzer *analyzer):
	analyzer(analyzer),
	initialized(false)
{
	ros::NodeHandle n;

	// Services
	this->initializeCameraService = n.advertiseService("initialize_camera", &Communicator::handleInitializeCameraService, this);

	// Subscribers
	this->pictureSendingActivationSubscriber = n.subscribe("picture_sending_activation", 1, &Communicator::handlePictureSendingActivation, this);

	// Publishers
	this->picturePublisher = n.advertise<camera_application::Picture>("picture", 1);
}

void Communicator::sendPicture(camera_application::Picture::_image_type &data, uint64_t timestamp)
{
	static uint32_t num = 0;

	camera_application::Picture msg;
	msg.imageNumber = num++;
	msg.timestamp = timestamp;
	msg.image = data;

	this->picturePublisher.publish(msg);
}

bool Communicator::handleInitializeCameraService(
		camera_application::InitializeCameraService::Request &req,
		camera_application::InitializeCameraService::Response &res)
{
	ROS_INFO("Received initialize_camera message.");
	if (this->initialized) {
		ROS_ERROR("initialize_camera called twice, ignoring.");
		res.error = 1;
	} else {
		this->initialized = true;
		this->hsvColorRanges = req.hsvColorRanges;
		this->quadCopterIds = req.quadCopterIds;
		res.error = 0;
	}
	return true;
}

void Communicator::handlePictureSendingActivation(
		const camera_application::PictureSendingActivation::Ptr &msg)
{
	ROS_INFO("Received picture_sending_activation message. Setting %d to %d.", msg->ID, msg->active);
	if (analyzer->isStarted() && !msg->active) {
		analyzer->stop();
	}
	else if (!analyzer->isStarted() && msg->active) {
		analyzer->start();
	}
}
