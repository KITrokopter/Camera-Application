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

class CvImageProcessor : public ImageAnalyzer
{
private:
	IImageReceiver* calibrationImageReceiver;
	ICalibrationFinishedListener* calibrationFinishedListener;
	ITrackerDataReceiver* dataReceiver;
	IUndistortedImageReceiver *undistortedImageReceiver;
	
	std::vector<cv::Scalar> colorRanges;
	cv::Mat* intrinsicsMatrix;
	cv::Mat* distortionCoefficients;
	std::vector<cv::Mat*> calibrationImages;
	
	// Camera calibration.
	boost::thread* calibrationThread;
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
	/**
	 * Creates a new object. The visualTracker option is disabled using this constructor.
	 * 
	 * @param imageSource A Kinect connection.
	 * @param dataReceiver An object to send the calculated quadcopter positions to.
	 */
	CvImageProcessor(CvKinect* imageSource, ITrackerDataReceiver* dataReceiver);
	
	/**
	 * Creates a new object.
	 * 
	 * @param imageSource A Kinect connection.
	 * @param dataReceiver An object to send the calculated quadcopter positions to.
	 * @param visualTracker If true, an OpenCV window will show the results of the tracking process live.
	 * @param useMaskedImage If the color masked image or the original image should be shown.
	 *                       Only has an effect if useMaskedImage is set to true.
	 */
	CvImageProcessor(CvKinect* imageSource, ITrackerDataReceiver* dataReceiver, bool showCameraImage, bool showMaskedImage);
	
	void processImage(cv::Mat* image, long int time);
	
	/**
	 * Sets the intrinsics matrix for the undistortion of the camera images. The matrix will be copied internally.
	 * 
	 * @param intrinsicsMatrix A 3x3 matrix containing double values.
	 */
	void setIntrinsicsMatrix(cv::Mat* intrinsicsMatrix);
	
	/**
	 * Sets the distortion coefficients for the undistortion of the camera images. The matrix will be copied internally.
	 * 
	 * @param distortionCoefficients A 5x1 matrix containing double values.
	 */
	void setDistortionCoefficients(cv::Mat* distortionCoefficients);
	
	/**
	 * Returns the intrinsics matrix of this camera. This can either be the result of a calibration, or a previously set intrinsics matrix.
	 * The caller has to delete the result.
	 * 
	 * @return A 3x3 matrix containing double values.
	 */
	cv::Mat* getIntrinsicsMatrix();
	
	/**
	 * Returns the distortion coefficients of this camera. This can either be the result of a calibration, or previously set distortion coefficients.
	 * The caller has to delete the result.
	 * 
	 * @return A 5x1 matrix containing double values.
	 */
	cv::Mat* getDistortionCoefficients();
	
	// Camera calibration.
	
	/**
	 * The function that calibrates the camera. You can use {@link startCalibration} if you want a non-blocking method.
	 */
	void calibrateCamera();
	
	/**
	 * Deletes all images that were collected during calibration.
	 */
	void clearCalibrationImages();
	
	/**
	 * Waits for the calibration to finish. Only works, if the {@link startCalibration} method was used.
	 */
	void waitForCalibration();
	
	/**
	 * Returns an undistorted version of the given image.<br />
	 * Doesn't work if the camera wasn't calibrated before and no custom calibration data has
	 * been set via the {@link setIntrinsicsMatrix} and {@link setDistortionCoefficients} methods.
	 * The caller has to delete the result.
	 * 
	 * @param inputImage The image to undistort.
	 */
	cv::Mat* undistortImage(cv::Mat* inputImage);
	
	/*
	 * Method: startCalibration
	 * 
	 * Starts the camera calibration process.
	 * 
	 * Parameters:
	 * imageAmount - The amount of images to use for the calibration.
	 * imageDelay - The delay between two images.
	 * boardWidth - The amount of corners on the x axis of the board.
	 * boardHeight - The amount of corners on the y axis of the board.
	 * boardRectangleWidth - The width of one board rectangle.
	 * boardRectangleHeight - The height of one board rectangle.
	 */
	void startCalibration(int imageAmount, int imageDelay, int boardWidth, int boardHeight, float boardRectangleWidth, float boardRectangleHeight, IImageReceiver* calibrationImageReceiver, ICalibrationFinishedListener* calibrationFinishedListener);
	
	/**
	 * Aborts the calibration. The calibration data it used before the calibration will be kept.
	 */
	void abortCalibration();
	
	/**
	 * Returns true if the camera is calibrated. That means, the camera has a not-null intrinsics matrix and a not-null distortion coefficients matrix.
	 * 
	 * @return True if the camera is calibrated, false otherwise.
	 */
	bool isCalibrated();
	
	// Tracking
	
	/**
	 * Adds a quadcopter to be tracked.
	 * 
	 * @param addQuadcopter The quadcopter.
	 */
	void addQuadcopter(QuadcopterColor* qc);
	
	/**
	 * Removes the quadcopter with the given id.
	 * 
	 * @param id The id.
	 */
	void removeQuadcopter(int id);
	
	/**
	 * Removes all quadcopters.
	 */
	void removeAllQuadcopters();
	
	/**
	 * Starts the tracking process.
	 */
	void startTracking();
	
	/**
	 * Stops the tracking process.
	 */
	void stopTracking();
	
	/**
	 * Sets the receiver for the calculated quadcopter vectors.
	 * 
	 * @param receiver The receiver.
	 */
	void setDataReceiver(ITrackerDataReceiver* receiver);
	
	/**
	 * Sets the receiver for the undistorted images.
	 * 
	 * @param receiver The receiver.
	 */
	void setUndistortedImageReceiver(IUndistortedImageReceiver *receiver);
	
	/**
	 * Default destructor, ensures that all threads are stopped.
	 */
	~CvImageProcessor();
};

#endif // CV_IMAGE_PROCESSOR_HPP
