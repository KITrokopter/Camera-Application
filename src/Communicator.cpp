#include "Communicator.hpp"

#include <ctime>
#include <iostream>
#include <string>
#include "api_application/Announce.h"

Communicator::Communicator(CvKinect *device, CvImageProcessor *analyzer) :
	device(device),
	analyzer(analyzer),
	initialized(false)
{
	ros::NodeHandle n;

	// Advertise myself to API
	ros::ServiceClient announceClient = n.serviceClient<api_application::Announce>("announce");

	api_application::Announce announce;
	announce.request.type = 0; // 0 means camera module

	if (announceClient.call(announce)) {
		id = announce.response.id;

		if (id == ~0 /* -1 */) {
			ROS_ERROR("Error! Got id -1");
			exit(1);
		} else   {
			ROS_INFO("Camera module successfully announced. Got id %d", id);
		}
	} else   {
		ROS_ERROR("Error! Could not announce myself to API!");
		exit(2);
	}

	// Services
	std::stringstream ss;
	ss << "InitializeCameraService" << id;

	this->initializeCameraService = n.advertiseService(ss.str(), &Communicator::handleInitializeCameraService, this);

	// Subscribers
	this->pictureSendingActivationSubscriber =
	    n.subscribe("PictureSendingActivation", 1, &Communicator::handlePictureSendingActivation,
	                this);
	this->calibrateCameraSubscriber = n.subscribe("CalibrateCamera", 100, &Communicator::handleCalibrateCamera, this);
	this->cameraCalibrationDataSubscriber =
	    n.subscribe("CameraCalibrationData", 4, &Communicator::handleCameraCalibrationData,
	                this);
	this->systemSubscriber = n.subscribe("System", 2, &Communicator::handleSystem, this);

	// Publishers
	this->picturePublisher = n.advertise<camera_application::Picture>("Picture", 1);
	this->rawPositionPublisher = n.advertise<camera_application::RawPosition>("RawPosition", 1);
	this->cameraCalibrationDataPublisher = n.advertise<camera_application::CameraCalibrationData>(
	    "CameraCalibrationData", 4);

	// Listen to the camera.
	device->addImageReceiver(this);
	analyzer->setUndistortedImageReceiver(this);

	pictureNumber = 0;
	pictureSendingActivated = false;
	firstPictureReceived = false;
}

void Communicator::receiveImage(cv::Mat *image, long int time, int type)
{
	if (!firstPictureReceived) {
		ROS_DEBUG("Received image. Kinect seems to be working");
		firstPictureReceived = true;
	}

	if (pictureSendingActivated && !analyzer->isCalibrated()) {
		camera_application::Picture::_image_type data;

		for (size_t i = 0; i < (640 * 480 * 3); i++) {
			data[i] = image->data[i];
		}

		this->sendPicture(data, (uint64_t)time, type);
	}

	delete image;
}

void Communicator::receiveUndistortedImage(cv::Mat *image, long int time)
{
	if (pictureSendingActivated && analyzer->isCalibrated()) {
		camera_application::Picture::_image_type data;

		for (size_t i = 0; i < (640 * 480 * 3); i++) {
			data[i] = image->data[i];
		}

		this->sendPicture(data, (uint64_t)time, 0);
	}

	delete image;
}

void Communicator::receiveTrackingData(cv::Scalar direction, int quadcopterId, long int time)
{
	camera_application::RawPosition pos;
	pos.ID = this->id;
	pos.timestamp = time;
	pos.xPosition = direction[0];
	pos.yPosition = direction[1];
	pos.quadcopterId = quadcopterId;

	rawPositionPublisher.publish(pos);
}

void Communicator::calibrationFinished(cv::Mat *intrinsicsMatrix, cv::Mat *distortionCoefficients)
{
	camera_application::CameraCalibrationData msg;
	msg.createdByCamera = true;

	for (int i = 0; i < 9; i++) {
		msg.intrinsics[i] = intrinsicsMatrix->at<double>(i);
	}

	for (int i = 0; i < 4; i++) {
		msg.distortion[i] = distortionCoefficients->at<double>(i);
	}

	cameraCalibrationDataPublisher.publish(msg);

	analyzer->clearCalibrationImages();
}

void Communicator::sendPicture(camera_application::Picture::_image_type &data, uint64_t timestamp, int type)
{
	camera_application::Picture msg;
	msg.ID = this->id;
	msg.imageNumber = pictureNumber++;
	msg.timestamp = timestamp;
	msg.image = data;
	msg.calibrationImage = (type == 1);

	this->picturePublisher.publish(msg);
}

bool Communicator::handleInitializeCameraService(
    camera_application::InitializeCameraService::Request &req,
    camera_application::InitializeCameraService::Response &res)
{
	if (this->initialized)
		analyzer->removeAllQuadcopters();

	ROS_INFO("Initializing camera %u.", id);
	this->initialized = true;
	this->hsvColorRanges = req.hsvColorRanges;
	this->quadCopterIds = req.quadCopterIds;

	// Set quadcopters
	for (int i = 0; i < this->quadCopterIds.size(); i++) {
		QuadcopterColor *color =
		    new QuadcopterColor(hsvColorRanges[i * 2], hsvColorRanges[(i * 2) + 1], quadCopterIds[i]);
		analyzer->addQuadcopter(color);
		ROS_DEBUG("Added quadcopter: %s", color->toString().c_str());
	}

	res.error = 0;

	return true;
}

void Communicator::handlePictureSendingActivation(
    const camera_application::PictureSendingActivation::Ptr &msg)
{
	if (msg->ID != this->id && !msg->all)
		return;

	pictureSendingActivated = msg->active;

	if (pictureSendingActivated)
		ROS_DEBUG("Picture sending activated");
	else
		ROS_DEBUG("Picture sending deactivated");
}

void Communicator::handleCalibrateCamera(
    const camera_application::CalibrateCamera::Ptr &msg)
{
	if (msg->ID == this->id) {
		ROS_DEBUG("Received calibration order for me");

		analyzer->startCalibration(msg->imageAmount, msg->imageDelay, msg->boardWidth, msg->boardHeight,
		                           msg->boardRectangleWidth, msg->boardRectangleHeight, this,
		                           this);
	}
}

void Communicator::handleCameraCalibrationData(
    const camera_application::CameraCalibrationData::Ptr &msg)
{
	if (msg->ID == this->id && msg->createdByCamera == false) {
		cv::Mat intrinsicsMatrix(cv::Size(3, 3), CV_64F);
		cv::Mat distortionCoefficients(cv::Size(5, 1), CV_64F);

		for (int i = 0; i < 9; i++) {
			intrinsicsMatrix.at<double>(i) = msg->intrinsics[i];
		}

		for (int i = 0; i < 4; i++) {
			distortionCoefficients.at<double>(i) = msg->distortion[i];
		}

		distortionCoefficients.at<double>(4) = 0;

		analyzer->setIntrinsicsMatrix(&intrinsicsMatrix);
		analyzer->setDistortionCoefficients(&distortionCoefficients);
	}
}

void Communicator::handleSystem(
    const api_application::System::Ptr &msg)
{
	if (msg->command == 1) {
		// Start

		analyzer->startTracking();
	} else if (msg->command == 2) {
		// Stop

		analyzer->stopTracking();
		ros::shutdown();
	}
}

