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

	/*
	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
	std::cerr << "ERROR>>>>Member function drawCurve is not implemented!" << std::endl;
	flag = true;
	}
	//=========================================================================

	// Robustness: make sure there is at least two control point: start and end points
	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	// Note that you must draw the whole curve at each frame, that means connecting line segments between each two points on the curve
	*/

	Point prevPoint = controlPoints[0].position;
	Point outputPoint;
	bool check;

	for (int i = 0; i < window; i++) {

		float t = (i / (float)window);
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
	/*
	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
	std::cerr << "ERROR>>>>Member function sortControlPoints is not implemented!" << std::endl;
	flag = true;
	}
	//=========================================================================
	*/

	CurvePoint C0 = controlPoints[0];
	CurvePoint C1 = controlPoints[1];
	CurvePoint C2 = controlPoints[2];
	CurvePoint C3 = controlPoints[3];

	// Create a position vector
	std::vector<Point> s;

	for (int i = 0; i < 4; i++) {

		s.push_back(controlPoints[i].position);

	}

	std::sort(s.begin(), s.end());

	for (int i = 0; i < 4; i++) {

		if (s[i] == C0.position) {
			controlPoints[i] = C0;
		}

		if (s[i] == C1.position) {
			controlPoints[i] = C1;
		}

		if (s[i] == C2.position) {
			controlPoints[i] = C2;
		}

		if (s[i] == C3.position) {
			controlPoints[i] = C3;
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

	/*
	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
	std::cerr << "ERROR>>>>Member function checkRobust is not implemented!" << std::endl;
	flag = true;
	}
	//=========================================================================
	*/

	int vectorLength = controlPoints.size();

	if (vectorLength < 2) {
		return false;
	}

	return true;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	/*
	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
	std::cerr << "ERROR>>>>Member function findTimeInterval is not implemented!" << std::endl;
	flag = true;
	}
	//=========================================================================
	*/

	// Find the index of the control point at time t or the interval t lies in.
	// Return the next index.

	int vectorLength = controlPoints.size();

	for (int i = 0; i < vectorLength - 1; i++) {

		if (controlPoints[i].time <= time < controlPoints[i + 1].time) {
			nextPoint = i + 1;
		}

	}

	return true;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;

	/*
	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
	std::cerr << "ERROR>>>>Member function useHermiteCurve is not implemented!" << std::endl;
	flag = true;
	}
	//=========================================================================
	*/

	// Calculate position at t = time on Hermite curve
	int i = nextPoint - 1;

	Point C0 = controlPoints[i].position;
	Point C1 = controlPoints[i + 1].position;

	Vector T0 = controlPoints[i].tangent;
	Vector T1 = controlPoints[i + 1].tangent;

	float f1 = 2 * pow(time, 3) - 3 * pow(time, 2) + 1;
	float f2 = -2 * pow(time, 3) + 3 * (time, 2);
	float f3 = pow(time, 3) - 2 * pow(time, 2) + time;
	float f4 = pow(time, 3) - pow(time, 2);

	newPosition = C0.operator*(f1) +C1.operator*(f2) +T0.operator*(f3) +T1.operator*(f4);

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