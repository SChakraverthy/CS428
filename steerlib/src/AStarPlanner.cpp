//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

namespace SteerLib
{
	AStarPlanner::AStarPlanner() {}

	AStarPlanner::~AStarPlanner() {}

	bool AStarPlanner::canBeTraversed(int id)
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x - OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z - OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i <= x_range_max; i += GRID_STEP)
		{
			for (int j = z_range_min; j <= z_range_max; j += GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				traversal_cost += gSpatialDatabase->getTraversalCost(index);

			}
		}

		if (traversal_cost > COLLISION_COST)
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		//TODO
		std::cout << "\nIn A*";

		// Set epsilon here.
		double epsilon = 1;

		// Uncomment a single line below to run the given algorithm.
		//return computePathWeightedAstar(agent_path, start, goal, epsilon, append_to_path);
		return computePathARAstar(agent_path, start, goal, epsilon, append_to_path);
	}

	/* Weighted A*/
	bool AStarPlanner::computePathWeightedAstar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, double epsilon, bool append_to_path) {

		// Create the open and closed sets.
		std::vector<AStarPlannerNode> openSet;
		std::vector<AStarPlannerNode> closedSet;

		// Add the start node to the open set.
		double g_start = 0;
		double h_start = calcEuclidianDistance(start, goal);
		// double h_start = calcManhattanDistance(start, goal);
		double f_start = epsilon * h_start;

		AStarPlannerNode* startNode = new AStarPlannerNode(start, g_start, f_start, NULL);
		openSet.push_back(*startNode);

		// Setup for the loop
		bool goalIsExpanded = false;
		
		// Debug statement to count number of expansions.
		//int count = 0;

		while (!goalIsExpanded) {

			// Get the node with the lowest f-value from the open set. Add to the closed set, remove from open set.
			AStarPlannerNode minNode = getNodeWithLowest_f(openSet);
			AStarPlannerNode* minNodeCopy = new AStarPlannerNode(minNode.point, minNode.g, minNode.f, minNode.parent);

			closedSet.push_back(minNode);

			for (std::vector<AStarPlannerNode>::iterator i = openSet.begin(); i != openSet.end(); i++){

				if (minNode.operator==(*i)) {

					openSet.erase(i);
					break;

				}

			}

			// Debug statements.
			//std::cout << count << ".: The minNode has point: " << minNode.point << "and f-value: " << minNode.f << std::endl;
			//count++;

			if (minNode.parent != NULL) {

				AStarPlannerNode* minParent = minNode.parent;
				std::cout << "The minNode parent has point: " << minParent->point << "and f-value: " << minParent->f << std::endl;

			}



			// Check if the node to be expanded is the goal. If so, then calculate path to it from start.
			if (goal.operator==(minNode.point)) {

				// Expanding the goal.
				goalIsExpanded = true;

				// Debug statements
				std::cout << std::endl;
				std::cout << "FOUND GOAL" << std::endl;
				//std::cout << "The goal point is: " << minNode.point << std::endl;

				
				reconstructPath(agent_path, goal, minNode);

				return true;
			}

			// Expand the minNode.
			std::vector<Util::Point> successors = getSuccessors(minNode);

			for (std::vector<Util::Point>::iterator i = successors.begin(); i != successors.end(); i++) {

bool alreadyVisited = false;
double old_f = DBL_MAX;
double old_g = DBL_MAX;
AStarPlannerNode* nodeInOpen = isInSet(*i, openSet);
AStarPlannerNode* nodeInClosed = isInSet(*i, closedSet);

// Ignore successor if in closed set.
if (nodeInClosed != NULL) {

	// Node in closed set.
	continue;

}

// Check if the successor is in open set already. If so, update old f, g values.
if (nodeInOpen != NULL) {

	// Already in open.
	alreadyVisited = true;
	old_f = nodeInOpen->f;
	old_g = nodeInOpen->g;

}

// Calculate new values.
double new_g = minNode.g + calculate_g(minNode.point, *i);
double new_h = calcEuclidianDistance(*i, goal);
//double new_h = calcManhattanDistance(*i, goal);
double new_f = new_g + (epsilon * new_h);

// Compare values and skip/update/add node as necessary.
if (new_f <= old_f) {

	if ((new_f == old_f) && (new_g > old_g)) {

		// Skip successor.
		continue;

	}

	if (alreadyVisited) {

		nodeInOpen->f = new_f;
		nodeInOpen->g = new_g;
		nodeInOpen->parent = minNodeCopy;
		continue;
	}
	else {

		AStarPlannerNode* successorNode = new AStarPlannerNode(*i, new_g, new_f, minNodeCopy);
		openSet.push_back(*successorNode);
		continue;
	}


}
else {

	continue;

}

			}

		}




		return false;
	}

	/* ARA* */
	bool AStarPlanner::computePathARAstar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, double epsilon, bool append_to_path) {

		// Create the open, closed, and inconsistent sets.
		std::vector<AStarPlannerNode> openSet;
		std::vector<AStarPlannerNode> closedSet;
		std::vector<AStarPlannerNode> inconSet;

		// Add the start node to the open set.
		double start_g = 0;
		double start_h = calcEuclidianDistance(start, goal);
		//double start_h = calcManhattanDistance(start, goal);
		double start_f = epsilon * start_h;

		AStarPlannerNode* startNode = new AStarPlannerNode(start, start_g, start_f, NULL);
		openSet.push_back(*startNode);

		// Initialize the goal f_value.
		double goal_f = DBL_MAX;

		// Calculate the current path for the given epsilon.

		// The initial minNode is the start node.
		AStarPlannerNode minNode = *startNode;
		AStarPlannerNode* minNodeCopy = new AStarPlannerNode(minNode.point, minNode.g, minNode.f, minNode.parent);
		
		while (goal_f > minNode.f) {

			// Debug Statement.
			//std::cout << "The minNode has point: " << minNode.point << " and f-value: " << minNode.f << std::endl;

			// Delete the minNode from the open set.

			for (std::vector<AStarPlannerNode>::iterator i = openSet.begin(); i != openSet.end(); i++){

				if (minNode.operator==(*i)) {

					openSet.erase(i);
					break;

				}

			}

			// Add the minNode to the closed set.
			closedSet.push_back(minNode);

			// Check to see if the node expanded is the goal. This solution is the current optimal solution.
			if (goal.operator==(minNode.point)) {

				std::cout << "FOUND THE GOAL" << std::endl;
				reconstructPath(agent_path, goal, minNode);
				return true;

			}


			// Get the successors of the minNode.
			std::vector<Util::Point> successors = getSuccessors(minNode);

			for (std::vector<Util::Point>::iterator i = successors.begin(); i != successors.end(); i++) {

				double old_g = DBL_MAX;
				double old_f = DBL_MAX;

				bool alreadyVisited = false;
				bool updateValues = false;

				// Check if the node was visited before.
				AStarPlannerNode* nodeInOpen = isInSet(*i, openSet);
				AStarPlannerNode* nodeInClosed = isInSet(*i, closedSet);
				AStarPlannerNode* nodeInIncon = isInSet(*i, inconSet);
				AStarPlannerNode* node = NULL;

				if (nodeInClosed != NULL) {

					alreadyVisited = true;
					node = nodeInClosed;

				}

				if (nodeInOpen != NULL) {

					alreadyVisited = true;
					old_g = nodeInOpen->g;
					old_f = nodeInOpen->f;
					node = nodeInOpen;

				}

				if (nodeInIncon != NULL) {


					alreadyVisited = true;
					old_g = nodeInIncon->g;
					old_f = nodeInIncon->f;
					node = nodeInClosed;
				}

				//std::cout << "MADE IT HERE" << std::endl;

				// Calculate new values
				double new_g = minNode.g + calculate_g(minNode.point, goal);
				double new_h = calcEuclidianDistance(*i, goal);
				//double new_h = calcManhattanDistance(*i, goal);
				double new_f = new_g + (epsilon * new_h);

				// Compare values to see if they need to be updated.
				if (new_f <= old_f) {

					if ((new_f == old_f) && (new_g > old_g)) {

						updateValues = false;

					}
					else {

						updateValues = true;

					}


				}

				// Only update or create a node for the successor if it's not already in closed.
				AStarPlannerNode* successorNode = NULL;
				if (!nodeInClosed) {

					if (alreadyVisited) {

						if (updateValues) {

							node->f = new_f;
							node->g = new_g;
							node->parent = minNodeCopy;
						}

						successorNode = node;


					}
					else {

						successorNode = new AStarPlannerNode(*i, new_g, new_f, minNodeCopy);

					}



				}
				else {

					successorNode = node;

				}

				// Only add to open or inconsistent sets if a node was updated or created.
				if (updateValues) {

					if (nodeInClosed == NULL) {

						// Insert the successor into the open set.
						openSet.push_back(*successorNode);

					}
					else {

						// Insert the successor into the inconsistent set.
						inconSet.push_back(*node);

					}


				}


			}

			// Calculate the minNode for the next iteration of the loop.
			minNode = getNodeWithLowest_f(openSet);
			minNodeCopy = new AStarPlannerNode(minNode.point, minNode.g, minNode.f, minNode.parent);

		}


		return false;
	}


	/* Finds and returns the node with the lowest f-value in the open set*/
	AStarPlannerNode AStarPlanner::getNodeWithLowest_f(std::vector<AStarPlannerNode> openSet) {

		double min_f = DBL_MAX;
		AStarPlannerNode* minNode;

		for (std::vector<AStarPlannerNode>::iterator i = openSet.begin(); i != openSet.end(); i++) {

			if (i->f < min_f) {

				min_f = i->f;
				minNode = &(*i);

			}


		}

		return *minNode;

	}

	std::vector<Util::Point> AStarPlanner::getSuccessors(AStarPlannerNode predecessor) {

		std::vector<Util::Point> neighbors;
		std::vector<Util::Point> successors;
		Util::Point pos = predecessor.point;

		// Vertical and horizontal directions.
		neighbors.push_back(Util::Point(pos.x + GRID_STEP, pos.y, pos.z));
		neighbors.push_back(Util::Point(pos.x - GRID_STEP, pos.y, pos.z));
		neighbors.push_back(Util::Point(pos.x, pos.y, pos.z + GRID_STEP));
		neighbors.push_back(Util::Point(pos.x, pos.y, pos.z - GRID_STEP));

		// Diagonal directions.
		neighbors.push_back(Util::Point(pos.x + GRID_STEP, pos.y, pos.z + GRID_STEP));
		neighbors.push_back(Util::Point(pos.x + GRID_STEP, pos.y, pos.z - GRID_STEP));
		neighbors.push_back(Util::Point(pos.x - GRID_STEP, pos.y, pos.z + GRID_STEP));
		neighbors.push_back(Util::Point(pos.x - GRID_STEP, pos.y, pos.z - GRID_STEP));

		// Check if the given neighbor is valid and add to the list of successors.
		for (std::vector<Util::Point>::iterator i = neighbors.begin(); i != neighbors.end(); i++) {

			int index = gSpatialDatabase->getCellIndexFromLocation(*i);

			if (canBeTraversed(index)) {

				// Add to the list of successors.
				successors.push_back(*i);

			}


		}

		return successors;

	}

	/* Heuristic: Euclidean Distance*/
	double AStarPlanner::calcEuclidianDistance(Util::Point p1, Util::Point p2) {

		return sqrt(pow(p1.x - p2.x, 2) + pow(p1.z - p2.z, 2));

	}

	/* Heuristic: Manhattan Distance */
	double AStarPlanner::calcManhattanDistance(Util::Point p1, Util::Point p2) {

		return abs(p1.x - p2.x) + abs(p1.z - p2.z);

	}

	/* Calculation for the g-value based on the change in x,z.*/
	double AStarPlanner::calculate_g(Util::Point p1, Util::Point p2) {

		double changeX = abs(p2.x - p1.x);
		double changeZ = abs(p2.z - p1.z);

		if ((changeX != 0 && changeZ == 0) || (changeX == 0 && changeZ != 0)) {
			return 1.0;
		}
		else {
			return sqrt(2.0);
		}


	}

	/* Reconstructs the path from the start to the goal.*/
	void AStarPlanner::reconstructPath(std::vector<Util::Point>& agent_path, Util::Point goal, AStarPlannerNode goalNode) {

		std::vector<Util::Point> backwardsPath;
		backwardsPath.push_back(goalNode.point);
		AStarPlannerNode* currNode = goalNode.parent;

		while (currNode != NULL) {

			backwardsPath.push_back(currNode->point);
			currNode = currNode->parent;

		}

		// Return the path from start to goal.
		for (std::vector<Util::Point>::reverse_iterator i = backwardsPath.rbegin(); i != backwardsPath.rend(); ++i) {

			agent_path.push_back(*i);

		}

		return;


	}

	AStarPlannerNode* AStarPlanner::isInSet(Util::Point pos, std::vector<AStarPlannerNode> set) {


		AStarPlannerNode* toReturn = NULL;

		for (std::vector<AStarPlannerNode>::iterator i = set.begin(); i != set.end(); i++) {


			if (pos.operator==(i->point)) {

				return &(*i);

			}

		}

		return toReturn;
	}
}

	