
#include "Communicator.hpp"

#include <ctime>

Communicator::Communicator(CvKinect *device, ImageAnalyzer *analyzer):
	device(device),
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

	// Listen to the camera.
	device->addImageReceiver(this);
}

void Communicator::receiveImage(cv::Mat* image, long int time)
{
	unsigned int notnull = 0;
	camera_application::Picture::_image_type data;
	for (size_t i = 0; i < (640 * 480); i++) {
		if (image->data[i]) notnull++;
		data[i] = image->data[i];
	}
	ROS_INFO("notnull: %u", notnull);
	this->sendPicture(data, (uint64_t)time);
	delete image;
}

void Communicator::sendPicture(camera_application::Picture::_image_type &data, uint64_t timestamp)
{
	static uint32_t num = 0;

	camera_application::Picture msg;
	msg.ID = this->id;
	msg.imageNumber = num++;
	msg.timestamp = timestamp;
	msg.image = data;

	this->picturePublisher.publish(msg);
}

bool Communicator::handleInitializeCameraService(
		camera_application::InitializeCameraService::Request &req,
		camera_application::InitializeCameraService::Response &res)
{
	if (this->initialized) {
		ROS_ERROR("initialize_camera called twice, ignoring.");
		res.error = 1;
	} else {
		ROS_INFO("Initializing camera %u.", req.ID);
		this->initialized = true;
		this->id = req.ID;
		this->hsvColorRanges = req.hsvColorRanges;
		this->quadCopterIds = req.quadCopterIds;
		res.error = 0;
	}
	return true;
}

void Communicator::handlePictureSendingActivation(
		const camera_application::PictureSendingActivation::Ptr &msg)
{
	if (msg->ID != this->id)
		return;
	if (analyzer->isStarted() && !msg->active) {
		ROS_INFO("Stopping image analyzer.");
		analyzer->stop();
	}
	else if (!analyzer->isStarted() && msg->active) {
		ROS_INFO("Starting image analyzer.");
		analyzer->start();
	}
}
