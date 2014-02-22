#ifndef QUADCOPTER_COLOR_HPP
#define QUADCOPTER_COLOR_HPP

#include <opencv2/core/core.hpp>
#include <string>

class QuadcopterColor
{
private:
	cv::Scalar minColor;
	cv::Scalar maxColor;
	int id;
	
public:
	QuadcopterColor(cv::Scalar minColor, cv::Scalar maxColor, int id);
	QuadcopterColor(uint8_t minHue, uint8_t maxHue, uint8_t minSaturation, uint8_t maxSaturation, uint8_t minValue, uint8_t maxValue, int id);
	QuadcopterColor(uint32_t min, uint32_t max, int id);
	cv::Scalar getMinColor();
	cv::Scalar getMaxColor();
	int getId();
	std::string toString();
};

#endif // QUADCOPTER_COLOR_HPP