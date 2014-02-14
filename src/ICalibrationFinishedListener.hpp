#ifndef I_CALIBRATION_FINISHED_LISTENER_HPP
#define I_CALIBRATION_FINISHED_LISTENER_HPP

#include <opencv2/core/core.hpp>

class ICalibrationFinishedListener {
public:
	/**
	 * Is called after the calibration of one camera is finished. The callee has to free the given matrices.
	 * 
	 * @param intrinsicsMatrix The resulting intrinsics matrix.
	 * @param distortionCoefficients The resulting distortion coefficients.
	 */
	virtual void calibrationFinished(cv::Mat* intrinsicsMatrix, cv::Mat* distortionCoefficients) = 0;
};

#endif // I_CALIBRATION_FINISHED_LISTENER_HPP