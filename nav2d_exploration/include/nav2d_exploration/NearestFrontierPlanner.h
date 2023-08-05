#ifndef NEARESTFRONTIERPLANNER_H_
#define NEARESTFRONTIERPLANNER_H_

#include <ros/ros.h>
#include <nav2d_navigator/ExplorationPlanner.h>

class NearestFrontierPlanner : public ExplorationPlanner
{
	public:
		NearestFrontierPlanner();
		~NearestFrontierPlanner();

		int findExplorationTarget(GridMap* map, unsigned int start, unsigned int &goal);


	private:
		typedef std::vector<unsigned int> Frontier;
		typedef std::vector<Frontier> FrontierList;

		FrontierList mFrontiers;  // Store frontier
		unsigned int mFrontierCells;
		std::vector<double> mPlan;  // Record the distance of all grid cells to current position
		unsigned int mOffset[8]; // Used to apply offset to current cell to get its neighbors

		bool mVisualizeFrontiers;

		ros::Publisher mFrontierPublisher;  // id = 0

		ros::Publisher finished_pub;


		void findFrontiers(GridMap* map, unsigned int start);
		void findCluster(GridMap* map, unsigned int startCell);
		void publishFrontier(GridMap* map, unsigned int& mFrontierCells, FrontierList& mFrontiers);

};

#endif // NEARESTFRONTIERPLANNER_H_
