#include "NearestFrontierPlanner.h"
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>

typedef std::multimap<double,unsigned int> Queue;
typedef std::pair<double,unsigned int> Entry;


int removeSmallUnknownCells(GridMap* map);
int removeSmallFrontiers(std::vector<std::vector<unsigned int>>& mFrontiers);

int mid_size = -1;


NearestFrontierPlanner::NearestFrontierPlanner()
{		
	ros::NodeHandle nh;
	mVisualizeFrontiers = true;
	mFrontierPublisher = nh.advertise<visualization_msgs::Marker>("/Navigator/frontiers", 1, true);
	ROS_INFO("NearestFrontierPlanner start!");
	finished_pub = nh.advertise<std_msgs::Bool>("/finished_topic", 10);
}

NearestFrontierPlanner::~NearestFrontierPlanner()
{
	
}

int NearestFrontierPlanner::findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal)
{

	// Frontier Calculation
    removeSmallUnknownCells(map);
	this->mFrontiers.clear();  
	this->mFrontierCells = 0;
	this->findFrontiers(map, start);   // Update mFrontiers, mFrontierCells, mPlan
	int numFrontier = removeSmallFrontiers(this->mFrontiers);
	this->mFrontierCells = 0;
	for(int k = 0; k < (int)this->mFrontiers.size(); k++){
		this->mFrontierCells += this->mFrontiers[k].size();
	}
	this->publishFrontier(map, mFrontierCells, mFrontiers);

	// Find the closest frontier and set as the target goal
	if (numFrontier <= 0){
		std_msgs::Bool msg;
		msg.data = true;
		finished_pub.publish(msg);
		return EXPL_FINISHED;
	}
	unsigned int target_index;
	double max_distance = std::numeric_limits<double>::max();
	for (int i = 0; i < this->mFrontiers.size(); i++){
		for (unsigned int cell: this->mFrontiers[i]){
			unsigned int numFreeNeighbor = map->getNumFreeNeighbors(cell);
			if(numFreeNeighbor < 4){  // Adjacent frontier cells are also free
				continue;
			}

			if (this->mPlan[cell] < max_distance) {
				max_distance = this->mPlan[cell];
				target_index = cell;
			}
		}
	}
	if (target_index >= 0){
		goal = target_index;
		return EXPL_TARGET_SET;
	}
	return EXPL_FAILED;

	// // Create some workspace for the wavefront algorithm
	// unsigned int mapSize = map->getSize();
	// double* plan = new double[mapSize];
	// for(unsigned int i = 0; i < mapSize; i++)
	// {
	// 	plan[i] = -1;
	// }
	
	// // Initialize the queue with the robot position
	// Queue queue;
	// Entry startPoint(0.0, start);
	// queue.insert(startPoint);
	// plan[start] = 0;
	
	// Queue::iterator next;
	// double distance;
	// double linear = map->getResolution();
	// bool foundFrontier = false;
	// int cellCount = 0;
	
	// // Do full search with weightless Dijkstra-Algorithm
	// while(!queue.empty())
	// {
	// 	cellCount++;
	// 	// Get the nearest cell from the queue
	// 	next = queue.begin();
	// 	distance = next->first;
	// 	unsigned int index = next->second;
	// 	queue.erase(next);
		
	// 	// Add all adjacent cells
	// 	if(map->isFrontier(index))
	// 	{
	// 		// We reached the border of the map, which is unexplored terrain as well:
	// 		foundFrontier = true;
	// 		goal = index;
	// 		break;
	// 	}else
	// 	{
	// 		unsigned int ind[4];

	// 		ind[0] = index - 1;               // left
	// 		ind[1] = index + 1;               // right
	// 		ind[2] = index - map->getWidth(); // up
	// 		ind[3] = index + map->getWidth(); // down
			
	// 		for(unsigned int it = 0; it < 4; it++)
	// 		{
	// 			unsigned int i = ind[it];
	// 			if(map->isFree(i) && plan[i] == -1)
	// 			{
	// 				queue.insert(Entry(distance+linear, i));
	// 				plan[i] = distance+linear;
	// 			}
	// 		}
	// 	}
	// }

	// ROS_DEBUG("Checked %d cells.", cellCount);	
	// delete[] plan;
	
	// if(foundFrontier)
	// {
	// 	return EXPL_TARGET_SET;
	// }else
	// {
	// 	if(cellCount > 50)
	// 		return EXPL_FINISHED;
	// 	else
	// 		return EXPL_FAILED;
	// }
}



int removeSmallUnknownCells(GridMap* map){
    // Each cell of the map may have three values, -1, 0, 100?
	unsigned int mapSize = map->getSize();
    std::vector<unsigned int> removeUnknown;
	for(unsigned int i = 0; i < mapSize; i++){
		if(map->isUnknown(i)){
            unsigned int freeNeighbor = map->getNumFreeNeighbors(i);
            if(freeNeighbor >= 7)
                removeUnknown.push_back(i);
        }
	}
	for(unsigned int& i: removeUnknown){
		map->setData(i, 0);
	}
    // ROS_DEBUG("%d Small Unknown cell is removed.", static_cast<int>(removeUnknown.size()));
    return 0;
}



/**
 * @brief Remove small frontiers from mFrontiers and return the number of the remaining.
 * 
 * @param mFrontiers 
 * @return int 
 */
int removeSmallFrontiers(std::vector<std::vector<unsigned int>>& mFrontiers){
	int numF = (int)mFrontiers.size();
	if(numF == 0){
		return numF;
	}

    std::vector<int> frontier_sizes;
    for(int k = 0; k < numF; k++){
        frontier_sizes.push_back((int)mFrontiers[k].size());
    }
    std::sort(frontier_sizes.begin(), frontier_sizes.end());
	ROS_DEBUG("Max-front Size: %d, Min-front Size: %d", frontier_sizes.back(), frontier_sizes.front());
	// mid_size will always be smaller than 10
	if(mid_size < 0){  // Initial value is -1
		mid_size = std::min((int)mFrontiers[numF/2].size(), 10);
	}else{
		mid_size = std::max(mid_size, std::min((int)mFrontiers[numF/2].size(), 10));
	}

	std::vector<int> removeIndices;
	for(int k = 0; k < (int)mFrontiers.size(); k++){
		if(mFrontiers[k].size() < mid_size){
			removeIndices.push_back(k);
		}
	}
	// Remove from back to front to avoid index change
	for(auto it = removeIndices.rbegin(); it != removeIndices.rend(); ++it){
		if(*it < numF){
			mFrontiers.erase(mFrontiers.begin() + *it);
		}
	}

	ROS_DEBUG("%d Small Frontiers are removed. %d remain.", (int)removeIndices.size(), numF - (int)removeIndices.size());

	if((int)mFrontiers.size() == 0){	
		ROS_WARN("No Frontiers in the map!");
	}
	return (int)mFrontiers.size();
}



/**
 * @brief Update mFrontiers and mPlan
 * 
 * @param map 
 * @param start 
 */
void NearestFrontierPlanner::findFrontiers(GridMap* map, unsigned int start){
	unsigned int mapSize = map->getSize();
	this->mPlan.clear();

	// Initialize map to be unknown
	for(unsigned int i = 0; i < mapSize; i++){
		this->mPlan.push_back(-1);
	}

	this->mOffset[0] = -1;					// left
	this->mOffset[1] =  1;					// right
	this->mOffset[2] = -map->getWidth();	// up
	this->mOffset[3] =  map->getWidth();	// down
	this->mOffset[4] = -map->getWidth() - 1;
	this->mOffset[5] = -map->getWidth() + 1;
	this->mOffset[6] =  map->getWidth() - 1;
	this->mOffset[7] =  map->getWidth() + 1;
	
	// Initialize the queue with the robot position
	Queue queue;
	Entry startPoint(0.0, start);
	queue.insert(startPoint);
	this->mPlan[start] = 0;
	
	Queue::iterator next;
	unsigned int index;
	double linear = map->getResolution();
	int cellCount = 0;  // Count of cells in the queue

	// Search for frontiers with wavefront propagation
	while(!queue.empty()) {
		cellCount++;
		// Get the nearest cell from the queue
		next = queue.begin();
		double distance = next->first;
		index = next->second;
		queue.erase(next);
		
		// Now continue 1st level WPA
		for(unsigned int it = 0; it < 4; it++) {
			unsigned int i = index + mOffset[it];
			if(i >= mapSize){
				continue;
			}
			if(this->mPlan[i] == -1 && map->isFree(i)){  // Only free cell will check whether is frontier or not
				// Check if it is a frontier cell
				if(map->isFrontier(i)){
					this->findCluster(map, i);   // Frontier will be added in this function
				}else{
					queue.insert(Entry(distance+linear, i));  // Add into queue to search for frontier
				}
				this->mPlan[i] = distance+linear;
			}
		}
	}
	return;
}


/**
 * @brief Given a frontier cell, expand it into a frontier cluster
 * 
 * @param map 
 * @param startCell 
 */
void NearestFrontierPlanner::findCluster(GridMap* map, unsigned int startCell)
{	
	Frontier front;
	
	// Initialize a new queue with the found frontier cell
	Queue frontQueue;
	frontQueue.insert(Entry(0.0, startCell));
	bool isBoundary = false;
	
	while(!frontQueue.empty())
	{
		// Get the nearest cell from the queue
		Queue::iterator next = frontQueue.begin();
		double distance = next->first;
		unsigned int index = next->second;
		unsigned int x, y;
		frontQueue.erase(next);
		
		// Check if it is a frontier cell by inspecting its neighboring 8 cells
		if(!map->isFrontier(index)) 
			continue;
		
		// Add it to current frontier
		front.push_back(index);
		mFrontierCells++;
		
		// Add all adjacent cells to queue
		for(unsigned int it = 0; it < 4; it++)
		{
			int i = index + mOffset[it];
			// Only insert free and initially unknown cell into the queue
			// If the i is not free, then it will not be considered as frontier
			// isFree means in current occupancy map, the cell is free
			if(map->isFree(i) && this->mPlan[i] == -1)  
			{
				this->mPlan[i] = distance + map->getResolution();
				frontQueue.insert(Entry(distance + map->getResolution(), i));
			}
		}
	}
	this->mFrontiers.push_back(front);
}



void NearestFrontierPlanner::publishFrontier(GridMap* map, unsigned int& mFrontierCells, FrontierList& mFrontiers){
	if(mVisualizeFrontiers)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time();
		marker.id = 0;
		marker.type = visualization_msgs::Marker::CUBE_LIST;
		marker.action = visualization_msgs::Marker::ADD;
		// marker.pose.position.x = map->getOriginX() + (map->getResolution() / 2);
		// marker.pose.position.y = map->getOriginY() + (map->getResolution() / 2);
		marker.pose.position.x = map->getOriginX();
		marker.pose.position.y = map->getOriginY();
		// marker.pose.position.z = map->getResolution() / 2;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = map->getResolution();
		marker.scale.y = map->getResolution();
		marker.scale.z = map->getResolution();
		marker.color.a = 0.5;
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;
		marker.points.resize(mFrontierCells);  // 为什么已经定义了pose，还能继续定义points和colors?
		marker.colors.resize(mFrontierCells);
		
		unsigned int p = 0;
		srand(1337);
		unsigned int x, y;
		
		for(unsigned int i = 0; i < mFrontiers.size(); i++)
		{				
			for(unsigned int j = 0; j < mFrontiers[i].size(); j++)
			{
				if(p < mFrontierCells)
				{
					if(!map->getCoordinates(x, y, mFrontiers[i][j]))
					{
						ROS_ERROR("[NearestFrontierPlanner] getCoordinates failed!");
						break;
					}
					marker.points[p].x = x * map->getResolution();
					marker.points[p].y = y * map->getResolution();
					marker.points[p].z = 0;
					
					marker.colors[p].r = 0.0;
					marker.colors[p].g = 0.0;
					marker.colors[p].b = 1.0;
					marker.colors[p].a = 0.6;
				}else
				{
					ROS_ERROR("[NearestFrontierPlanner] SecurityCheck failed! (Asked for %d / %d)", p, mFrontierCells);
				}
				p++;
			}
		}
		mFrontierPublisher.publish(marker);
	}
}