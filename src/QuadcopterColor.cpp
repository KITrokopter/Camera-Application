#include "QuadcopterColor.hpp"

QuadcopterColor::QuadcopterColor(cv::Scalar minColor, cv::Scalar maxColor, int id)
{
	this->minColor = minColor;
	this->maxColor = maxColor;
	this->id = id;
}

QuadcopterColor::QuadcopterColor(int minHue, int maxHue, int minSaturation, int maxSaturation, int minValue, int maxValue, int id)
{
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