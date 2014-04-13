#ifndef CV_IMAGE_PROCESSOR_HPP
#define CV_IMAGE_PROCESSOR_HPP

#include <vector>
#include <opencv2/core/core.hpp>
#include <boost/thread.hpp>

#include "ImageAnalyzer.hpp"
#include "Tracker.hpp"
#include "ITrackerDataReceiver.hpp"
#include "IImageReceiver.hpp"
#include "CvKinect.hpp"
#include "ICalibrationFinishedListener.hpp"
#include "IUndistortedImageReceiver.hpp"

/**
 * Handles camera calibration, image undistortion, and passes undistorted images to the trackers for the differen quadcopters.
 *
 * @author Sebastian Schmidt
 */
class CvImageProcessor : public ImageAnalyzer {
private:
	IImageReceiver *calibrationImageReceiver;
	ICalibrationFinishedListener *calibrationFinishedListener;
	ITrackerDataReceiver *dataReceiver;
	IUndistortedImageReceiver *undistortedImageReceiver;

	std::vector<cv::Scalar> colorRanges;
	cv::Mat *intrinsicsMatrix;
	cv::Mat *distortionCoefficients;
	std::vector<cv::Mat*> calibrationImages;

	// Camera calibration.
	boost::thread *calibrationThread;
	volatile bool abortCalibrationFlag;
	volatile double calibrationError;
	int imageAmount;
	int imageDelay;
	int boardWidth;
	int boardHeight;
	float boardRectangleWidth;
	float boardRectangleHeight;

	// Tracking
	std::vector<Tracker*> trackers;
	bool isTracking;
	bool showCameraImage;
	bool showMaskedImage;

	// Methods
	std::vector<cv::Point3f>* createObjectPoints();

	std::vector<cv::Point2f>* createImagePoints();

public:

	CvImageProcessor(CvKinect *imageSource, ITrackerDataReceiver *dataReceiver);
	CvImageProcessor(CvKinect *imageSource, ITrackerDataReceiver *dataReceiver, bool showCameraImage,
	                 bool showMaskedImage);

	void processImage(cv::Mat *image, long int time);

	void setIntrinsicsMatrix(cv::Mat *intrinsicsMatrix);
	void setDistortionCoefficients(cv::Mat *distortionCoefficients);


	cv::Mat* getIntrinsicsMatrix();
	cv::Mat* getDistortionCoefficients();

	// Camera calibration.

	void calibrateCamera();
	void clearCalibrationImages();
	void waitForCalibration();
	cv::Mat* undistortImage(cv::Mat *inputImage);
	void startCalibration(int imageAmount, int imageDelay, int boardWidth, int boardHeight, float boardRectangleWidth,
	                      float boardRectangleHeight, IImageReceiver *calibrationImageReceiver,
	                      ICalibrationFinishedListener *calibrationFinishedListener);
	void abortCalibration();
	bool isCalibrated();

	// Tracking

	void addQuadcopter(QuadcopterColor *qc);
	void removeQuadcopter(int id);
	void removeAllQuadcopters();
	void startTracking();
	void stopTracking();
	void setDataReceiver(ITrackerDataReceiver *receiver);
	void setUndistortedImageReceiver(IUndistortedImageReceiver *receiver);

	~CvImageProcessor();
};

#endif // CV_IMAGE_PROCESSOR_HPP
