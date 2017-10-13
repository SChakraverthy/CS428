#include "obstacles/GJK_EPA.h"

// Helper function definitions. Need to be moved into header file.

bool GJK_Algo(const std::vector<Util::Vector>& polyA, const std::vector<Util::Vector>& polyB, std::vector<Util::Vector>& simplex);
Util::Vector supportFn(std::vector<Util::Vector> poly, Util::Vector d);
std::vector<Util::Vector> MinDiff(std::vector<Util::Vector> polyA, std::vector<Util::Vector> polyB);
bool containsOrigin(std::vector<Util::Vector>& simplex, Util::Vector& d);
void EPA_Algo(std::vector<Util::Vector> minDiff, std::vector<Util::Vector>& simplex, float& return_penetration_depth, Util::Vector& return_penetration_vector);
std::vector<Util::Vector> findEdge(std::vector<Util::Vector> simplex);

void testFn(std::vector<Util::Vector> polyA, std::vector<Util::Vector> polyB);


SteerLib::GJK_EPA::GJK_EPA()
{
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{

	// Determine if the shapes intersect using GJK.
	std::vector<Util::Vector> simplex;
	bool hasIntersection;

	return_penetration_depth = 0;
	return_penetration_vector = Util::Vector();

	hasIntersection = GJK_Algo(_shapeA, _shapeB, simplex);

	if (hasIntersection) {

		//Find intersection point and penetration vector using EPA.

		std::vector<Util::Vector> minDiff = MinDiff(_shapeA, _shapeB); //Calculate Minkowski Difference to pass into EPA.
		EPA_Algo(minDiff, simplex, return_penetration_depth, return_penetration_vector);

		return true;
	}

	return false; // There is no collision
}

bool GJK_Algo(const std::vector<Util::Vector> &polyA, const std::vector<Util::Vector> &polyB, std::vector<Util::Vector>& simplex) {

	std::vector<Util::Vector> minDiff, W;
	Util::Vector d, t, v, w;

	// Get the Minkoswki Difference for polyA and polyB.
	minDiff = MinDiff(polyA, polyB);

	int length = minDiff.size();

	t = minDiff[std::rand() % length]; // Gets a random point within Minkowski Difference.
	v = supportFn(minDiff, t); // Gets point on the boundary of the Minkowski Difference.
	d = v.operator-();

	W.push_back(v); // Start with one point in the simplex.

	while (true) {

		// Get support point using direction vector d.
		w = supportFn(minDiff, d);

		if (Util::dot(w, d) <= 0) {

			return false;

		}
		else {

			W.push_back(w);

			// Check if the simplex contains the origin.
			if (containsOrigin(W, d)) {

				simplex = W;
				return true;

			}

		}

	}
}

Util::Vector supportFn(std::vector<Util::Vector> poly, Util::Vector d) {

	float highest = -FLT_MAX;
	float dotProduct;
	Util::Vector supportPoint, currPoint;

	int polyLength = poly.size();

	for (int i = 0; i < polyLength; i++) {

		currPoint = poly[i];
		dotProduct = Util::dot(currPoint, d);

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

bool containsOrigin(std::vector<Util::Vector>& simplex, Util::Vector& d) {

	// Get the last point added to the simplex and compute A0.
	Util::Vector A = simplex[simplex.size() - 1];
	Util::Vector A0 = A.operator-();

	// Check the type of simplex have.

	if (simplex.size() == 2) {

		// Get the point added before A. Calculate AB.
		Util::Vector B = simplex[0];
		Util::Vector AB = B.operator-(A);

		// Get the dot product of AB and A0.
		float dotProd = Util::dot(AB, A0);

		if (dotProd > 0) {

			// Set the new direction perpendicular to the simplex.
			d = Util::cross(Util::cross(AB, A0), AB);

		}
		else {

			// New search direction is A0.
			d = A0;

		}

		return false;

	}
	else if (simplex.size() == 3) {

		Util::Vector B = simplex[1];
		Util::Vector C = simplex[0];

		Util::Vector AB = B.operator-(A);
		Util::Vector AC = C.operator-(A);
		Util::Vector ABC = Util::cross(AB, AC);
		Util::Vector ABCxAC = Util::cross(Util::cross(AB, AC), AC);
		Util::Vector ABCxAB = Util::cross(Util::cross(AC, AB), AB);

		if (Util::dot(ABCxAC, A0) > 0) {

			// Remove B
			simplex[1] = A;
			simplex.pop_back();

			if (Util::dot(AC, A0)) {

				d = Util::cross(Util::cross(AC, A0), AC);

			}
			else {

				if (Util::dot(AB, A0)) {

					d = Util::cross(Util::cross(AB, A0), AB);

				}
				else {

					d = A0;

				}

			}

			return false;

		}
		else {

			if (Util::dot(ABCxAB, A0) > 0) {

				// Remove C from simplex.
				simplex[0] = B;
				simplex[1] = A;
				simplex.pop_back();

				if (Util::dot(AB, A0)) {

					d = Util::cross(Util::cross(AB, A0), AB);

				}
				else {

					d = A0;

				}

				return false;

			}

			return true;


		}

	}

	return false;
}

void EPA_Algo(std::vector<Util::Vector> minDiff, std::vector<Util::Vector>& simplex, float& return_penetration_depth, Util::Vector& return_penetration_vector) {

	while (true) {

		//Find the edge of the simplex closest to the origin.
		std::vector<Util::Vector> closestEdge = findEdge(simplex);
		Util::Vector A = closestEdge[0];
		Util::Vector B = closestEdge[1];
		Util::Vector AB = B - A;

		// Find a support point in the direction normal to the edge away from the origin.
		Util::Vector edge_normal = Util::normalize(Util::cross(Util::cross(AB, A), AB));
		Util::Vector supportPoint = supportFn(minDiff, edge_normal);

		float dist = Util::dot(supportPoint, edge_normal);
		float edge_dist = Util::dot(edge_normal, A);

		if (dist - edge_dist <= 0.0001) {

			// Found the MTV.
			return_penetration_depth = edge_dist;
			return_penetration_vector = edge_normal;
			return;

		}
		else {

			int k;

			// Add the support point to the simplex.
			for (int i = 0; i < simplex.size(); i++) {

				if (i + 1 == simplex.size()) {
					// Reached last vertice in simplex.
					k = 0;
				}
				else {
					k = i + 1;
				}

				if (simplex[i] == closestEdge[0] && simplex[k] == closestEdge[1]) {

					// Found the edge in the simplex.
					std::vector<Util::Vector>::iterator iterator = (simplex.begin() + k);
					simplex.insert(iterator, supportPoint);

				}


			}



		}

	}
}

std::vector<Util::Vector> findEdge(std::vector<Util::Vector> simplex) {

	std::vector<Util::Vector> closestEdge;
	Util::Vector A, B, E, N, norm_N;
	Util::Vector firstPoint, secondPoint;
	float edge_distance = FLT_MAX;

	int k;

	for (int i = 0; i < simplex.size(); i++) {

		if (i + 1 == simplex.size()) {
			// Reached last vertice in simplex.
			k = 0;
		}
		else {
			k = i + 1;
		}

		A = simplex[i];
		B = simplex[k];
		E = B.operator-(A);

		N = Util::cross(Util::cross(E, A), E);
		norm_N = Util::normalize(N);

		float d = Util::dot(norm_N, A);

		if (d < edge_distance) {

			firstPoint = A;
			secondPoint = B;
			edge_distance = d;

		}
	}

	closestEdge.push_back(firstPoint);
	closestEdge.push_back(secondPoint);

	return closestEdge;
}
