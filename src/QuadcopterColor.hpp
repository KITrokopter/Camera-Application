#ifndef QUADCOPTER_COLOR_HPP
#define QUADCOPTER_COLOR_HPP

#include <opencv2/core/core.hpp>

class QuadcopterColor
{
private:
	cv::Scalar minColor;
	cv::Scalar maxColor;
	int id;
	
public:
	QuadcopterColor(cv::Scalar minColor, cv::Scalar maxColor, int id);
	QuadcopterColor(int minHue, int maxHue, int minSaturation, int maxSaturation, int minValue, int maxValue, int id);
	cv::Scalar getMinColor();
	cv::Scalar getMaxColor();
	int getId();
};

#endif // QUADCOPTER_COLOR_HPP