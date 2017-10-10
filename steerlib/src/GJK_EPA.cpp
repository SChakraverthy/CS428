#include "obstacles/GJK_EPA.h"

// Helper function definitions. Need to be moved into header file.

bool GJK_Algo(const std::vector<Util::Vector>& polyA, const std::vector<Util::Vector>& polyB);
Util::Vector supportFn(std::vector<Util::Vector> poly, Util::Vector d);
std::vector<Util::Vector> MinDiff(std::vector<Util::Vector> polyA, std::vector<Util::Vector> polyB);
std::tuple<std::vector<Util::Vector>, Util::Vector> containsOrigin(std::vector<Util::Vector> simplex, Util::Vector A);

SteerLib::GJK_EPA::GJK_EPA()
{
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	// Determine if the shapes intersect using GJK.
	std::vector<Util::Vector> simplex;
	bool hasIntersection;

	hasIntersection = GJK_Algo(_shapeA, _shapeB);

	if (hasIntersection) {

		//Find intersection point and penetration vector using EPA.
		//EPA_Algo(return_penetration_depth, return_penetration_vector);
		return true;
	}

	return false; // There is no collision
}

/*** Note: The return value is currently set to bool but needs to return the simplex and bool when implementation is complete.*/
bool GJK_Algo(const std::vector<Util::Vector> &polyA, const std::vector<Util::Vector> &polyB) {

	// 1: Compute the Minkowski Difference of A and B
	std::vector<Util::Vector> minDiff = MinDiff(polyA, polyB);

	// 2: Choose arbitrary point v
	Util::Vector v;
	int length = minDiff.size();

	if (length > 0) {

		v = minDiff[std::rand() % length];

	}

	// 3: Define empty set W
	std::vector<Util::Vector> W;

	// Determine initial search direction vector.
	Util::Vector d = v.operator*(-1);

	// 4: Loop
	while (true) {

		// 5: Get support point w using support function.
		Util::Vector w = supportFn(minDiff, d);
		
		// 6: If w approximate to v, then return
		Util::Vector norm_w = w.operator/(w.lengthSquared());
		Util::Vector norm_v = v.operator/(v.lengthSquared());
		
		/*** Probably needs to be changed. ***/
		if (norm_w == norm_v) {

			return true;

		}
		else
		{
			// 7: Otherwise add w to W
			W.push_back(w);

			// 8: If convex hull of W contains origin, shapes intersect. Return.

		}


	/*To-Do:
	 9: Otherwise find point in simplex closest to the origin.

	 10: Remove points from W that do not contribute to the closest point approximation.

	 11: Repeat until algorithm converges. */
	}
	return false;
}

Util::Vector supportFn(std::vector<Util::Vector> poly, Util::Vector d) {

	float highest, dotProduct;
	Util::Vector supportPoint;
	Util::Vector currPoint;
	int polyLength = poly.size();

	for (int i = 0; i < polyLength; i++) {

		currPoint = poly[i];
		dotProduct = (currPoint.x * d.x) + (currPoint.z * d.z);

		if (dotProduct > highest) {

			highest = dotProduct;
			supportPoint = currPoint;

		}

	}

	return supportPoint;

}

std::vector<Util::Vector> MinDiff(std::vector<Util::Vector> polyA, std::vector<Util::Vector> polyB) {

	int length_A = polyA.size();
	int length_B = polyB.size();

	std::vector<Util::Vector> minDiff;
	Util::Vector diff;

	for (int i = 0; i < length_A; i++) {

		for (int j = 0; j < length_B; j++) {

			diff = polyA[i].operator-(polyB[j]);

			if (std::find(minDiff.begin(), minDiff.end(), diff) != minDiff.end()) {
				// Point already found in the Minkowski Difference.
				continue;
			}
			else {
				// Found new point in the Minkowski Difference.
				minDiff.push_back(diff);
			}

		}

	}

	return minDiff;

}