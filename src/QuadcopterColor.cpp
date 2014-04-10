#include "QuadcopterColor.hpp"

#include <sstream>

QuadcopterColor::QuadcopterColor(cv::Scalar minColor, cv::Scalar maxColor, int id)
{
	this->minColor = minColor;
	this->maxColor = maxColor;
	this->id = id;
}

QuadcopterColor::QuadcopterColor(uint8_t minHue, uint8_t maxHue, uint8_t minSaturation, uint8_t maxSaturation,
                                 uint8_t minValue, uint8_t maxValue,
                                 int id)
{
	this->minColor = cv::Scalar(minHue, minSaturation, minValue);
	this->maxColor = cv::Scalar(maxHue, maxSaturation, maxValue);
	this->id = id;
}

QuadcopterColor::QuadcopterColor(uint32_t min, uint32_t max, int id)
{
	uint8_t minHue = (min >> 16) & 0xFF;
	uint8_t maxHue = (max >> 16) & 0xFF;
	uint8_t minSaturation = (min >> 8) & 0xFF;
	uint8_t maxSaturation = (max >> 8) & 0xFF;
	uint8_t minValue = min & 0xFF;
	uint8_t maxValue = max & 0xFF;

	this->minColor = cv::Scalar(minHue, minSaturation, minValue);
	this->maxColor = cv::Scalar(maxHue, maxSaturation, maxValue);
	this->id = id;
}

cv::Scalar QuadcopterColor::getMinColor()
{
	return minColor;
}

cv::Scalar QuadcopterColor::getMaxColor()
{
	return maxColor;
}

int QuadcopterColor::getId()
{
	return id;
}

std::string QuadcopterColor::toString()
{
	std::stringstream ss;
	ss << "id: " << id << " H: " << minColor.val[0] << "-" << maxColor.val[0]
	   << " S: " << minColor.val[1] << "-" << maxColor.val[1]
	   << " V: " << minColor.val[2] << "-" << maxColor.val[2];
	return ss.str();
}