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
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id ) 
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
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
		std::cout << "In A" << std::endl;

		// Weighted A*
		float epsilon = 1;
		computePathWeightedAstar(agent_path, start, goal, epsilon, append_to_path);


		return false;
	}


	bool AStarPlanner::computePathWeightedAstar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, float epsilon, bool append_to_path) {

		// Check
		std::cout << "In Weighted A*" << std::endl;

		// Create open and closed lists
		std::vector<AStarPlannerNode> openSet;
		std::vector<AStarPlannerNode> closedSet;

		// Add the starting node to the open set.
		double start_g = 0;
		double start_h = calcEuclidianDistance(start, goal);
		//double start_h = calcManhattanDistance(start, goal);
		double start_f = start_g + start_h*epsilon;

		AStarPlannerNode* startNode = new AStarPlannerNode(start, start_g, start_f, NULL);
		openSet.push_back(*startNode);

		while (!openSet.empty()) {

			// Get the node in the open set with the lowest f-value.
			AStarPlannerNode minNode = getNodeWithLowest_f(openSet);
			std::cout << "The lowest f value is for the node with position" << minNode.point << "and f value" << minNode.f << std::endl;

			// Delete this node from the open set and add it to the closed set.
			closedSet.push_back(minNode);

			for (std::vector<AStarPlannerNode>::iterator i = openSet.begin(); i != openSet.end(); ++i) {

				int min_index = gSpatialDatabase->getCellIndexFromLocation(minNode.point);
				int curr_index = gSpatialDatabase->getCellIndexFromLocation(i->point);

				if (min_index == curr_index) {

					// Found the node to delete.
					openSet.erase(i);
					break;

				}

			}

			// Generate the list of successor indices.
			std::vector<int> successors = getSuccessors(minNode);

			for (std::vector<int>::iterator i = successors.begin(); i != successors.end(); i++) {

				int successor_index = *i;
				Point successor_pos;
				gSpatialDatabase->getLocationFromIndex(*i, successor_pos);

				// Check to see if have reached the goal.
				if (successor_pos.operator==(goal)) {

					std::cout << "GOAL FOUND" << std::endl;
					return true;

				}

				// Check to see if the successor is in the closed set.
				bool isInClosedSet = false;

				for (std::vector<AStarPlannerNode>::iterator j = closedSet.begin(); j != closedSet.end(); j++){
					
					int closed_index = gSpatialDatabase->getCellIndexFromLocation(j->point);

					if (successor_index == closed_index) {

						isInClosedSet = true;
						break;

					}

				}

				if (isInClosedSet) {
					continue;
				}

				// Check if the successor is already in the open set or not.
				bool isInOpenSet = false;
				AStarPlannerNode* successorNode;
				
				for (std::vector<SteerLib::AStarPlannerNode>::iterator j = openSet.begin(); j != openSet.end(); j++) {

					int open_index = gSpatialDatabase->getCellIndexFromLocation(j->point);

					if (successor_index == open_index) {

						isInOpenSet = true;
						successorNode = &(*j);
						break;

					}

				}

				// Calculate the new f, g, and h values.
				double new_g = calculate_g(minNode.point, successor_pos);
				double new_h = calcEuclidianDistance(successor_pos, goal);
				//double new_h = calcManhattanDistance(successor_pos, goal);
				double new_f = new_g + new_h * epsilon;


				if (isInOpenSet) {

					// Compare the f-values and update if necessary.
					if (successorNode->f == new_f) {

						// Compare the g values and update if necessary.
						if (new_g < successorNode->g) {

							successorNode->f = new_f;
							successorNode->g = new_g;
							successorNode->parent = &minNode;
							continue;

						}

					}
					else if (successorNode->f > new_f) {

						successorNode->f = new_f;
						successorNode->g = new_g;
						successorNode->parent = &minNode;
						break;

					}
				}
				else {

					// Must add the successor to the open set.
					AStarPlannerNode* node = new AStarPlannerNode(successor_pos, new_g, new_f, &minNode);
					openSet.push_back(*node);
				}

			}

			printSet(openSet);
			break;

		}

		return false;
	}


	/*
		Given the open set of nodes, returns the node with the lowest f-value to be used in the path planning algorithm.
	*/

	SteerLib::AStarPlannerNode AStarPlanner::getNodeWithLowest_f(std::vector<SteerLib::AStarPlannerNode> openSet) {

		// Search the open set for the node with the lowest f value.
		double minVal = DBL_MAX;
		SteerLib::AStarPlannerNode* minNode;

		for (std::vector<SteerLib::AStarPlannerNode>::iterator i = openSet.begin(); i != openSet.end(); i++) {

			SteerLib::AStarPlannerNode currNode = *i;

			if (currNode.f < minVal) {

				minVal = currNode.f;
				minNode = &currNode;

			}
		}

		return *(minNode);

	}

	/*
		Given a node as a predecessor, returns all valid successor indices of the node such that the successor nodes are traversable.
	*/

	std::vector<int> AStarPlanner::getSuccessors(SteerLib::AStarPlannerNode predecessor) {

		std::vector<int> successors;
		std::vector<Util::Point> neighbors;
		Util::Point pos = predecessor.point;

		// Compute the neighboring points on the grid.

		// Vertical and horizontal directions.
		neighbors.push_back(Util::Point(pos.x + GRID_STEP, 0, pos.z));
		neighbors.push_back(Util::Point(pos.x - GRID_STEP, 0, pos.z));
		neighbors.push_back(Util::Point(pos.x, 0, pos.z + GRID_STEP));
		neighbors.push_back(Util::Point(pos.x, 0, pos.z - GRID_STEP));

		// Diagonal directions.
		neighbors.push_back(Util::Point(pos.x + GRID_STEP, 0, pos.z + GRID_STEP));
		neighbors.push_back(Util::Point(pos.x + GRID_STEP, 0, pos.z - GRID_STEP));
		neighbors.push_back(Util::Point(pos.x - GRID_STEP, 0, pos.z + GRID_STEP));
		neighbors.push_back(Util::Point(pos.x - GRID_STEP, 0, pos.z - GRID_STEP));

		// For each cell that is valid, add to the list of successors.

		for (std::vector<Util::Point>::iterator i = neighbors.begin(); i != neighbors.end(); i++) {

			// Check if the given cell is valid. If so, add index to list of successors.
			int index = gSpatialDatabase->getCellIndexFromLocation(*i);
			if (canBeTraversed(index)) {

				successors.push_back(index);

			}
		}

		return successors;

	}

	/*
		Given a variable to store the agent path, the start node, the parent node of the goal node, and the goal, reconstructs the path from start to goal by starting
		from the goal and using the parent pointers to get the backwards path from goal to start. This is then reversed and set to the agent's path from start to goal.
	*/
	void AStarPlanner::reconstructPath(std::vector<Util::Point>& agent_path, SteerLib::AStarPlannerNode startNode, SteerLib::AStarPlannerNode goalParentNode, Util::Point goal) {

		// Create and initialize the backwards reconstructed path
		std::vector<Util::Point> backwardsPath;
		backwardsPath.push_back(goal);

		// Set the current node to add to the backwards path to the goal node's parent.
		SteerLib::AStarPlannerNode* currNode = &goalParentNode;

		while (true) {

			backwardsPath.push_back(currNode->point);

			// Have reached the start node. Backwards path has been constructed.
			if ((*currNode).operator==(startNode)) {
				break;
			}

			currNode = currNode->parent;

		}

		// Return the path from start to goal.
		for (std::vector<Util::Point>::reverse_iterator i = backwardsPath.rbegin(); i != backwardsPath.rend(); ++i) {
			
			agent_path.push_back(*i);

		}

		return;
	}

	/*
		Code to debug. Delete later.
	*/
	void AStarPlanner::printSet(std::vector<SteerLib::AStarPlannerNode> set) {

		std::cout << "The points in the open set are:" << std::endl;

		for (std::vector<SteerLib::AStarPlannerNode>::iterator i = set.begin(); i != set.end(); i++) {

			std::cout << i->point << std::endl;

		}

		return;
	}

	/*
		Heuristic: Euclidian Distance
	*/
	double AStarPlanner::calcEuclidianDistance(Util::Point p1, Util::Point p2) {

		return sqrt(pow(p1.x - p2.x, 2) + pow(p1.z - p2.z, 2));

	}

	/*
		Heuristic: Manhattan Distance
	*/
	double AStarPlanner::calcManhattanDistance(Util::Point p1, Util::Point p2) {

		return abs(p1.x - p2.x) + abs(p1.z - p2.z);

	}

	/*
		Calculate g value of the current position using the previous position.
	*/
	double AStarPlanner::calculate_g(Util::Point prevPos, Util::Point newPos) {

		double change_x = abs(newPos.x - prevPos.x);
		double change_z = abs(newPos.z - prevPos.z);

		if (change_x != 0 && change_z == 0) {

			return change_x;

		}
		else if (change_x == 0 && change_z != 0) {

			return change_z;

		}

		return sqrt(pow(change_x, 2) + pow(change_z, 2));
	}

}