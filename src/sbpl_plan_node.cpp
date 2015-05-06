//the node planning the discrete waypoint
#include <ros/ros.h>
#include <sbpl_plan/SbplPlanner.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sbpl_plan");
  	ros::NodeHandle n("~");

  	SbplPlanner planner;

  	if (!planner.initialize(n))
  	{
    	ROS_ERROR("%s: failed to initialize sbpl planner",
              ros::this_node::getName().c_str());
    	return EXIT_FAILURE;
  	}

  	ros::spin();

  	return EXIT_SUCCESS;
}