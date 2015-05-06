#ifndef SBPL_PLANNER_H
#define SBPL_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_utils/GeometryUtils.h>

class SbplPlanner
{
public:
	SbplPlanner();
	~SbplPlanner();

	bool initialize(const ros::NodeHandle& n);

private:
	bool loadParameters(const ros::NodeHandle& n);
	bool registerCallbacks(const ros::NodeHandle& n);

  	void fan1Callback(const nav_msgs::Odometry::ConstPtr& msg);
  	void fan2Callback(const nav_msgs::Odometry::ConstPtr& msg);
  	void quadCallback(const nav_msgs::Odometry::ConstPtr& msg);

  	void planTimerCallback(const ros::TimerEvent& e);

  	//publisher and subscriber
  	ros::Subscriber fan1_sub;
  	ros::Subscriber fan2_sub;
  	ros::Subscriber quad_sub;

  	ros::Publisher plan_pub;
  	ros::Timer plan_timer;

  	bool fan1_set;
  	bool fan2_set;
  	bool quad_set;

  	//node name
  	std::string name;

  	//fan and quad position
  	geometry_utils::Vec3 fan1_pos;
  	geometry_utils::Vec3 fan2_pos;
  	geometry_utils::Vec3 quad_pos;

  	//parameters
  	double rate;

};

#endif