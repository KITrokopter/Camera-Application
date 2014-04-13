#include "QuadcopterColor.hpp"

#include <sstream>

/**
 * Creates a new QuadcopterColor with the given parameters.
 *
 * @param minColor The lower interval borders.
 * @param maxColor The upper interval borders.
 * @param id The quadcopter id.
 */
QuadcopterColor::QuadcopterColor(cv::Scalar minColor, cv::Scalar maxColor, int id)
{
	this->minColor = minColor;
	this->maxColor = maxColor;
	this->id = id;
}

/**
 * Creates a new QuadcopterColor with the given parameters.
 *
 * @param minHue The minimum hue value.
 * @param maxHue The maximum hue value.
 * @param minSaturation The minimum saturation value.
 * @param maxSaturation The maximum saturation value (Ignored by tracker).
 * @param minValue The minimum "value" value.
 * @param maxValue The maximum "value" value.
 * @param id The quadcopter id.
 */
QuadcopterColor::QuadcopterColor(uint8_t minHue, uint8_t maxHue, uint8_t minSaturation, uint8_t maxSaturation,
                                 uint8_t minValue, uint8_t maxValue, int id)
{
	this->minColor = cv::Scalar(minHue, minSaturation, minValue);
	this->maxColor = cv::Scalar(maxHue, maxSaturation, maxValue);
	this->id = id;
}

/**
 * Creates a new QuadcopterColor with the given encoded color ranges.
 *
 * @param min The encoded lower interval borders.
 * @param max The encoded upper interval borders.
 * @param id The quadcopter id.
 */
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

/**
 * Returns the lower interval borders.
 *
 * @return The lower interval borders.
 */
cv::Scalar QuadcopterColor::getMinColor()
{
	return minColor;
}

/**
 * Returns the upper interval borders.
 *
 * @return The upper interval borders.
 */
cv::Scalar QuadcopterColor::getMaxColor()
{
	return maxColor;
}

/**
 * Returns the quadcopter id.
 *
 * @return The quadcopter id.
 */
int QuadcopterColor::getId()
{
	return id;
}

/**
 * Returns a string representation of the QuadcopterColor.
 *
 * @return A string representation of the QuadcopterColor.
 */
std::string QuadcopterColor::toString()
{
	std::stringstream ss;
	ss << "id: " << id << " H: " << minColor.val[0] << "-" << maxColor.val[0]
	   << " S: " << minColor.val[1] << "-" << maxColor.val[1]
	   << " V: " << minColor.val[2] << "-" << maxColor.val[2];
	return ss.str();
}