#ifndef I_CALIBRATION_FINISHED_LISTENER_HPP
#define I_CALIBRATION_FINISHED_LISTENER_HPP

#include <opencv2/core/core.hpp>

class ICalibrationFinishedListener {
public:
	virtual void calibrationFinished(cv::Mat* intrinsicsMatrix, cv::Mat* distortionCoefficients) = 0;
};

#endif // I_CALIBRATION_FINISHED_LISTENER_HPP