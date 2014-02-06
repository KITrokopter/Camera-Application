
#include "Communicator.hpp"

#include <ctime>

Communicator::Communicator(CvKinect *device, CvImageProcessor *analyzer):
	device(device),
	analyzer(analyzer),
	initialized(false)
{
	ros::NodeHandle n;

	// TODO: get id by announcing to api.
	
	// Services
	// TODO: Add id to service name.
	this->initializeCameraService = n.advertiseService("InitializeCameraService", &Communicator::handleInitializeCameraService, this);

	// Subscribers
	this->pictureSendingActivationSubscriber = n.subscribe("PictureSendingActivation", 1, &Communicator::handlePictureSendingActivation, this);

	// Publishers
	this->picturePublisher = n.advertise<camera_application::Picture>("Picture", 1);
	this->rawPositionPublisher = n.advertise<camera_application::RawPosition>("RawPosition", 1);

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

void Communicator::receiveTrackingData(cv::Scalar direction, int id, long int time)
{
	camera_application::RawPosition pos;
	pos.ID = id;
	pos.timestamp = time;
	pos.xPosition = direction[0];
	pos.yPosition = direction[1];
	pos.quadcopterId = id;
	
	rawPositionPublisher.publish(pos);
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
		
		// Set quadcopters
		for (int i = 0; i < this->hsvColorRanges.size(); i++)
		{
			QuadcopterColor* color = new QuadcopterColor(hsvColorRanges[i * 2], hsvColorRanges[(i * 2) + 1], quadCopterIds[i]);
			analyzer->addQuadcopter(color);
		}
		
		res.error = 0;
	}
	return true;
}

void Communicator::handlePictureSendingActivation(
		const camera_application::PictureSendingActivation::Ptr &msg)
{
	if (msg->ID != this->id && msg->ID != 0)
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
