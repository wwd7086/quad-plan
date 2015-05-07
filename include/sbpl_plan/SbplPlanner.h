#ifndef SBPL_PLANNER_H
#define SBPL_PLANNER_H

#include <ros/ros.h>
#include <sbpl_plan/grid2D.h>
#include <nav_msgs/Odometry.h>
#include <geometry_utils/GeometryUtils.h>
#include <sbpl/headers.h>

class SbplPlanner
{
public:
	SbplPlanner();
	~SbplPlanner();

	bool initialize(const ros::NodeHandle& n);

private:
	bool loadParameters(const ros::NodeHandle& n);
	bool registerCallbacks(const ros::NodeHandle& n);
	
	// subsribe obstacle and start pos information
  	void fan1Callback(const nav_msgs::Odometry::ConstPtr& msg);
  	void fan2Callback(const nav_msgs::Odometry::ConstPtr& msg);
  	void quadCallback(const nav_msgs::Odometry::ConstPtr& msg);

  	// publish path information
  	void planTimerCallback(const ros::TimerEvent& e);

  	//publisher and subscriber
  	ros::Subscriber fan1_sub;
  	ros::Subscriber fan2_sub;
  	ros::Subscriber quad_sub;

  	ros::Publisher plan_pub;
  	ros::Timer plan_timer;

  	// the map
  	Grid2D *map;

  	// it the value set?
  	bool fan1_set;
  	bool fan2_set;
  	bool quad_set;

  	//node name
  	std::string name;

  	//fan,quad,goal position
  	geometry_utils::Vec3 fan1_pos;
  	geometry_utils::Vec3 fan2_pos;
  	geometry_utils::Vec3 quad_pos;
  	geometry_utils::Vec3 goal_pos;

  	//parameters
  	double rate;
  	float grid_size;
  	float fan_R;
  	bool bsearchuntilfirstsolution;
    bool bforwardsearch;
    double initialEpsilon;
    unsigned char obsthresh;
    double allocated_time_secs;

    //sbpl
    EnvironmentNAV2D environment_nav2D;
    SBPLPlanner* planner;
	MDPConfig MDPCfg;
};

#endif