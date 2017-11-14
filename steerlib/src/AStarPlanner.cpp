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
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );
				
			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		//TODO
		std::cout<<"In A*" << std::endl;

		// Create open and closed lists.
		std::vector<SteerLib::AStarPlannerNode> openSet;
		std::vector<SteerLib::AStarPlannerNode> closedSet;

		// Add the starting node to the open set.
		double start_h = calcEuclidianDistance(start, goal);
		//double start_h = calcManhattanDistance(start, goal);
		SteerLib::AStarPlannerNode* startNode = new AStarPlannerNode(start, 0.0, start_h , NULL);
		openSet.push_back(*startNode);

		printSet(openSet);

		while (!openSet.empty()) {

			// Search for the node in the open set with the lowest f-value and remove from the open set.
			SteerLib::AStarPlannerNode minNode = getNodeWithLowest_f(openSet);

			for (std::vector<SteerLib::AStarPlannerNode>::iterator i; i != openSet.end(); i++) {

				int min_index = gSpatialDatabase->getCellIndexFromLocation(minNode.point);
				int curr_index = gSpatialDatabase->getCellIndexFromLocation(i->point);

				if (min_index == curr_index) {
					// Found the node to delete in the open set.
					openSet.erase(i);
				}
			}

			printSet(openSet);

			// Generate the list of successor indices.
			std::vector<int> successors = getSuccessors(minNode);

			for (std::vector<int>::iterator i = successors.begin(); i != successors.end(); i++) {

				int successor_index = *i;
				Util::Point successor_pos;
				gSpatialDatabase->getLocationFromIndex(*i, successor_pos);

				// Check to see if have reached the goal.
				if (successor_pos.operator==(goal)) {

					reconstructPath(agent_path, *startNode, minNode, goal);
					return true;

				}

				// Compute the f, g, and h values

				// If Euclidean Distance:
				double new_g = minNode.g + calcEuclidianDistance(minNode.point, successor_pos);
				double new_h = calcEuclidianDistance(successor_pos, goal);
				double new_f = new_g + new_h; // For weighted A*, would calculate new f with given epsilon.
				
				// If Manhattan Distance: Implement below.
				//double new_g = minNode.g + calcManhattanDistance(minNode.point, successor_pos);
				//double new_h = calcManhattanDistance(successor_pos, goal);
				//double new_f = new_g + new_h; // For weighted A*, would calculate new f with given epsilon.
				
				// Check to see if the successor is in the openSet or closedSet.

				bool isInClosedSet = false;

				for (std::vector<SteerLib::AStarPlannerNode>::iterator j = closedSet.begin(); j != closedSet.end(); j++) {

					if (successor_pos.operator==(j->point)) {

						isInClosedSet = true;

					}


				}

				if (isInClosedSet) {
					break;
				}

				// Compare f values if the node is already in the open set.
				for (std::vector<SteerLib::AStarPlannerNode>::iterator j = openSet.begin(); j != openSet.end(); j++) {



					if (successor_index == gSpatialDatabase->getCellIndexFromLocation(j->point)) {

						// The successor is already in the open set. Compare f values.
						if (new_f < j->f ) {

							// Keep old values.
						
						}



					}


				}


				// The successor was neither in the open or closed list. Add to the open list with the computed values.
				SteerLib::AStarPlannerNode* node = new AStarPlannerNode(successor_pos, new_g, new_f, &minNode);
				openSet.push_back(*node);


			}

			closedSet.push_back(minNode);


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
		Given a node as a predecessor, returns all valid successors of the node such that the nodes are traversable.
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

			// Check if the given cell is valid.
			int index = gSpatialDatabase->getCellIndexFromLocation(*i);
			if (canBeTraversed(index)) {

				// The neighboring cell with the given index is traversable.
				// Add the index to the list of successors.
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

}