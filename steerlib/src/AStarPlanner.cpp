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
		double epsilon = 1;
		return computePathWeightedAstar(agent_path, start, goal, epsilon, append_to_path);

	}

	/* Weighted A*/
	bool AStarPlanner::computePathWeightedAstar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, double epsilon, bool append_to_path) {

		// Create the open and closed sets.
		std::vector<AStarPlannerNode> openSet;
		std::vector<AStarPlannerNode> closedSet;

		// Add the start node to the open set.
		double start_g = 0;
		double start_h = calcEuclidianDistance(start, goal);
		//double start_h = calcManhattanDistance(start, goal);
		double start_f = start_g + (epsilon*start_h);

		AStarPlannerNode*  startNode = new AStarPlannerNode(start, start_g, start_f, NULL);
		openSet.push_back(*startNode);

		while (!openSet.empty()) {

			// Find the node from the open set with the minimum f-value.
			AStarPlannerNode minNode = getNodeWithLowest_f(openSet);
			
			//std::cout << "The node with the lowest f-value is at position: " << minNode.point << " with f-value: " << minNode.f << std::endl;

			Util::Point currPoint = minNode.point;

			// Find and remove minNode from the open set.
			for (std::vector<AStarPlannerNode>::iterator i = openSet.begin(); i != openSet.end(); i++) {

				if (minNode.operator==(*i)) {

					openSet.erase(i);
					break;
				}

			}

			// Add minNode to the closed set.
			closedSet.push_back(minNode);


			// Check if the minNode is the goal node.
			if (goal.operator==(currPoint)) {

				std::cout << std::endl;
				std::cout << "FOUND THE GOAL" << std::endl;
				std::cout << std::endl;

				//reconstructPath(agent_path, goal, minNode);

				return true;

			}

			// Expand the minNode.
			std::vector<Util::Point> successors = getSuccessors(minNode);
			
			for (std::vector<Util::Point>::iterator i = successors.begin(); i != successors.end(); i++) {

				Util::Point successor_pos = *i;

				// Check if the successor is in the closed set.
				for (std::vector<AStarPlannerNode>::iterator j = closedSet.begin(); j != closedSet.end(); j++) {

					if (successor_pos.operator==(j->point)) {

						// Skip successor
						continue;

					}

				}

				AStarPlannerNode* successorNode = NULL;

				// Check if the successor is in the open set.
				for (std::vector<AStarPlannerNode>::iterator j = openSet.begin(); j != openSet.end(); j++) {

					if (successor_pos.operator==(j->point)) {

						successorNode = &(*j);

					}


				}

				// Calculate new f, g, and h values.
				double new_g = minNode.g + calculate_g(minNode.point, successor_pos);
				double new_h = calcEuclidianDistance(successor_pos, goal);
				//double new_h = calcManhattanDistance(successor_pos, goal);
				double new_f = new_g + (epsilon * new_h);


				if (successorNode == NULL) {

					// Successor node was not in the open set. Add.
					AStarPlannerNode* node = new AStarPlannerNode(successor_pos, new_g, new_f, &minNode);
					openSet.push_back(*node);
					continue;
				}
				else {

					// Update successor node if necessary.
					double old_f = successorNode->f;
					double old_g = successorNode->g;

					if (new_f <= old_f) {

						if (new_f == old_f) {

							if (new_g >= old_g) {
								continue;
							}

						}

						// Update
						successorNode->g = new_g;
						successorNode->f = new_f;
						successorNode->parent = &minNode;
					}


				}



			}


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

		while (true) {

			if (currNode == NULL) {
				// Can't go back any further.
				break;
			}

			backwardsPath.push_back(currNode->point);
			currNode = currNode->parent;

		}

		// Return the path from start to goal.
		for (std::vector<Util::Point>::reverse_iterator i = backwardsPath.rbegin(); i != backwardsPath.rend(); ++i) {

			agent_path.push_back(*i);

		}

		return;


	}
}