//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
// Copyright (c) 2015 Mahyar Khayatkhoei
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

using namespace Util;

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI

	// Robustness: make sure there is at least two control point: start and end points
	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	// Note that you must draw the whole curve at each frame, that means connecting line segments between each two points on the curve

	if (!checkRobust()) {

		return;

	}

	Point prevPoint = controlPoints[0].position;
	Point outputPoint;
	float t;
	bool check;

	for (int i = 0; i < window; i++) {

		t = (i / (float)window);
		check = calculatePoint(outputPoint, t);
		DrawLib::drawLine(prevPoint, outputPoint, curveColor, curveThickness);
		prevPoint = outputPoint;
	}

	check = calculatePoint(outputPoint, 1);
	DrawLib::drawLine(prevPoint, outputPoint, curveColor, curveThickness);

	return;
#endif
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{

	int length = controlPoints.size();

	std::vector<CurvePoint> controlCopy;
	std::vector<float> timeVector;

	for (int i = 0; i < length; i++) {

		controlCopy.push_back(controlPoints[i]);
		timeVector.push_back(controlPoints[i].time);

	}

	std::sort(timeVector.begin(), timeVector.end());

	for (int i = 0; i < length; i++) {

		for (int j = 0; j < length; j++) {

			if (timeVector[i] == controlCopy[j].time) {

				controlPoints[i] = controlCopy[j];
				break;

			}

		}

	}

	return;
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
// Note that this function should return false if the end of the curve is reached, or no next point can be found
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	// Note that nextPoint is an integer containing the index of the next control point
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve given the next control point (nextPoint)
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
	
	/* Check to make sure that there are at least 2 control points.*/
	int length = controlPoints.size();

	if (length < 2) {

		return false;

	}

	return true;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	
	int length = controlPoints.size();

	for (int i = 0; i < length - 1; i++) {

		if (controlPoints[i].time < time <= controlPoints[i + 1].time) {

			nextPoint = i + 1;
			return true;
		}

	}

	return false;


	//return true;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;

	// Calculate position at t = time on Hermite curve

	int i = nextPoint - 1; // Index of the last control point.

	Point P0 = controlPoints[i].position;
	Point P1 = controlPoints[i + 1].position;
	Vector T0 = controlPoints[i].tangent;
	Vector T1 = controlPoints[i + 1].tangent;

	float f1 = 2 * pow(time, 3) - 3 * pow(time, 2) + 1;
	float f2 = -2 * pow(time, 3) + 3 * (time, 2);
	float f3 = pow(time, 3) - 2 * pow(time, 2) + time;
	float f4 = pow(time, 3) - pow(time, 2);

	newPosition = P0.operator*(f1) + P1.operator*(f2) + T0.operator*(f3) + T1.operator*(f4);

	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;

	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function useCatmullCurve is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================

	// Calculate position at t = time on Catmull-Rom curve

	// Return result
	return newPosition;
}