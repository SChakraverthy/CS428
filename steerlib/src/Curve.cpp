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

	// Calculates the number of steps we will use to create the curve.
	int length = controlPoints.size();
	int numSteps = (int)controlPoints[length - 1].time / window;

	Point prevPoint = controlPoints[0].position; // Grabs the position of the first control point.
	Point outputPoint;
	float t;
	bool check; // Currently we do nothing with the return value. calculatePoint() returns a bool values, so we will likely need to use this later to error check.

	for (int i = 1; i < numSteps; i++) {
		
		t = i * (float)window;
		
		/*Debugging Statements:
		std::cout << "The time is: " << t << std::endl;
		std::cout << std::endl;
		*/

		check = calculatePoint(outputPoint, t);
		DrawLib::drawLine(prevPoint, outputPoint, curveColor, curveThickness);
		prevPoint = outputPoint;
	}

	// Currently choosing to ignore the final segment until the rest of it works.

	return;
#endif
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
	
	int length = controlPoints.size();

	// This function creates a time vector that holds the time attributes for the control points.
	// Also creates a copy of the control points vector that will be used in sorting the control points.

	std::vector<CurvePoint> controlCopy;
	std::vector<float> timeVector;

	for (int i = 0; i < length; i++) {

		controlCopy.push_back(controlPoints[i]);
		timeVector.push_back(controlPoints[i].time);

	}

	/* Debugging statements
	std::cout << "The control points vector before sorting is: " << std::endl;
	for (int i = 0; i < length; i++) {

	std::cout << controlPoints[i].position << std::endl ;

	}
	*/

	// Sort the time vector
	std::sort(timeVector.begin(), timeVector.end());

	// For each value in the sorted time vector, find the corresponding control point using the control copy vector.
	// Reorder the original control points vector accordingly.
	for (int i = 0; i < length; i++) {

		for (int j = 0; j < length; j++) {

			if (timeVector[i] == controlCopy[j].time) {

				controlPoints[i] = controlCopy[j];
				break;

			}

		}

	}

	/* Debugging statements
	std::cout << "The control points vector after sorting is: " << std::endl;

	for (int i = 0; i < length; i++) {

	std::cout << controlPoints[i].position << std::endl;

	}
	*/

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
	// Check to make sure that there are at least 2 control points.
	int length = controlPoints.size();

	if (length < 2) {

		return false;

	}

	return true;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	
	// Search the control points to find the time interval that the current time lies in.
	// Use iterator i to index through the control points and find the next control point
	// the curve should follow at the current time passed as a parameter. If this value
	// matches the interval, nextPoint is set to i + 1.
	std::cout << "---Time Interval curr Time: " << time << std::endl;

	int length = controlPoints.size();

	for (int i = 0; i < length - 1; i++) {

		if (controlPoints[i].time <= time && time < controlPoints[i + 1].time) {

			nextPoint = i + 1;
			std::cout << "i + 1 is: " << i + 1 << std::endl;

		}

	}


	return true;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;

	// Calculate position at t = time on Hermite curve

	int i = nextPoint - 1; // Index of the last control point.

	// Determine the 2 control points we are interpolating between and their tangent
	// vectors.
	Point P0 = controlPoints[i].position;
	Point P1 = controlPoints[i + 1].position;
	Vector T0 = controlPoints[i].tangent;
	Vector T1 = controlPoints[i + 1].tangent;

	// Calculate the position of the new point using the Hermite Equation.
	float f1 = 2 * pow(time, 3) - 3 * pow(time, 2) + 1;
	float f2 = -2 * pow(time, 3) + 3 * (time, 2);
	float f3 = pow(time, 3) - 2 * pow(time, 2) + time;
	float f4 = pow(time, 3) - pow(time, 2);

	newPosition = P0.operator*(f1) +P1.operator*(f2) +T0.operator*(f3) +T1.operator*(f4);

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