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

	Point prevPoint = controlPoints[0].position;
	Point outputPoint;
	float t;
	bool check; // Currently we do nothing with the return value. Error check later.

	for (int i = 1; i < numSteps; i++) {
		t = i * (float)window;
		check = calculatePoint(outputPoint, t);
		DrawLib::drawLine(prevPoint, outputPoint, curveColor, curveThickness);
		prevPoint = outputPoint;
	}

	// Draws the final segment for the hermite curve.
	DrawLib::drawLine(prevPoint, controlPoints[length - 1].position, curveColor, curveThickness);

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

	// Sorts the time vector.
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

	// Checks to make sure that there are at least 2 control points.
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
	// the curve should follow given the current time passed as a parameter. If this value
	// matches the interval, nextPoint is set to i + 1.

	int length = controlPoints.size();
	int i;

	for (i = 0; i < length /*- 1*/; i++) {

		if (controlPoints[i].time <= time && time < controlPoints[i + 1].time) {

			nextPoint = i + 1;

		}

	}

	if (i == length - 1) {
		return i;
	}


	return true;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;

	// Calculate position at t = time on Hermite curve

	int length = controlPoints.size();

	if (time == controlPoints[nextPoint].time) {

		return controlPoints[nextPoint].position;

	}

	int i = nextPoint - 1; // Index of the last control point.

	// These values are used to convert the time to a value between 0 and 1 so that
	// we can correctly interpolate the curve point.
	float k = controlPoints[i].time;
	float n = controlPoints[i + 1].time;

	float t = (time - k) / (n - k);

	// Calculation

	// Determine the two control points we are interpolating between and their tangent vectors.
	Point C0 = controlPoints[i].position;
	Point C1 = controlPoints[nextPoint].position;
	Vector T0 = controlPoints[i].tangent;
	Vector T1 = controlPoints[nextPoint].tangent;

	// Calculate the position of the new point using the Hermite Equation.

	float f1 = 2 * pow(t, 3) - 3 * pow(t, 2) + 1;
	float f2 = 3 * pow(t, 2) - 2 * pow(t, 3);
	float f3 = pow(t, 3) - 2 * pow(t, 2) + t;
	float f4 = pow(t, 3) - pow(t, 2);

	newPosition = (((C0.operator*(f1)).operator+(C1.operator*(f2))).operator+(T0.operator*(f3))).operator+(T1.operator*(f4));

	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;

	// Calculate position at t = time on catmull curve

	int length = controlPoints.size();

	if (time == controlPoints[nextPoint].time) {

		return controlPoints[nextPoint].position;

	}

	//Index of last control point
	int i = nextPoint - 1;

	// These values are used to convert the time to a value between 0 and 1 so that
	// we can correctly interpolate the curve point.
	float k = controlPoints[i].time;
	float n = controlPoints[i + 1].time;

	float t = (time - k) / (n - k);
	
	//catmull rom curve needs 4 control points, one before and one after
	//the desried curve segment between P1 and P2
	Point P0 = controlPoints[i].position;
	Point P1 = controlPoints[i+1].position;
	Point P2 = controlPoints[i+2].position;
	Point P3 = controlPoints[i+3].position;

	//trying to make the polynomial equation easier to perform operations on


	// Calculate position at t = time on Catmull-Rom curve
	//using a given equation:
	//newPosition = 0.5{ [2P1] + [time(P2 - P0)] + [time^2(2P0 - 5P1 + 4P2 - P3)] +	[time^3(3P1 - 3P2 + P3 - P0)] }

	//split into a bunch of points to make the math easier
	float f1 = 0.5f;
	float f2 = t;
	float f3 = pow(t, 2);
	float f4 = pow(t, 3);

	Point s1 = P1.operator*(2.0f);
	Point L1 = P0.operator*(-1.0f);
	Point s2 = P2.operator+(L1);
	Point s3 = s2.operator*(t);
	Point s4 = s1.operator+(s3);

	Point r1 = P0.operator*(2.0f);
	Point r2 = P1.operator*(5.0f);
	Point r3 = P2.operator*(4.0f);
	Point L2 = r2.operator*(-1.0f);
	Point r4 = r1.operator+(L2);
	Point r5 = r4.operator+(r3);
	Point L3 = P3.operator*(-1.0f);
	Point r6 = r5.operator+(L3);
	Point r7 = r6.operator*(f3);

	Point q1 = P1.operator*(3.0f);
	Point q2 = P2.operator*(3.0f);
	Point L4 = q2.operator*(-1.0f);
	Point q3 = q1.operator+(L4);
	Point q4 = q3.operator+(P3);
	Point L5 = P0.operator*(-1.0f);
	Point q5 = q4.operator+(L5);
	Point q6 = q5.operator*(f4);

	Point x1 = s4.operator+(r7);
	Point x2 = x1.operator+(q6);
	Point x3 = x2.operator*(f1);

	newPosition = x3;

	// Return result
	return newPosition;
}