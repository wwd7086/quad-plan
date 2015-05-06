//sbpl planner wrapper
#include <sbpl_plan/SbplPlanner.h>
#include <geometry_utils/GeometryUtilsROS.h>
#include <sbpl/headers.h>

namespace gu = geometry_utils;
namespace gr = gu::ros;

SbplPlanner::SbplPlanner(): 
	fan1_set(false), 
	fan2_set(false), 
	quad_set(false)
{
}

SbplPlanner::~SbplPlanner()
{
}

bool SbplPlanner::initialize(const ros::NodeHandle& n)
{
	name = ros::names::append(n.getNamespace(), "SbplPlanner");

	if (!loadParameters(n))
  	{
    	ROS_ERROR("%s: failed to load parameters", name.c_str());
    	return false;
 	}

  	if (!registerCallbacks(n))
  	{
    	ROS_ERROR("%s: failed to register callbacks", name.c_str());
    	return false;
  	}

  	return true;
}

bool SbplPlanner::loadParameters(const ros::NodeHandle& n)
{
	rate = 10;
	return true;
}

bool SbplPlanner::registerCallbacks(const ros::NodeHandle& n)
{
	ros::NodeHandle ln(n);
	fan1_sub = ln.subscribe("fan1/odom",10,&SbplPlanner::fan1Callback, this);
	fan2_sub = ln.subscribe("fan1/odom",10,&SbplPlanner::fan2Callback, this);
	quad_sub = ln.subscribe("odom",10,&SbplPlanner::quadCallback, this);
	plan_pub = ln.advertise<nav_msgs::Path>("plan_path",10,false);
	plan_timer = ln.createTimer(rate, &SbplPlanner::planTimerCallback, this);

	return true;
}

void SbplPlanner::fan1Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	fan1_pos = gr::fromROS(msg->pose.pose.position);
	fan1_set = true;
}

void SbplPlanner::fan2Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	fan2_pos = gr::fromROS(msg->pose.pose.position);
	fan2_set = true;
}

void SbplPlanner::quadCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	quad_pos = gr::fromROS(msg->pose.pose.position);
	quad_set = true;
}

void SbplPlanner::planTimerCallback(const ros::TimerEvent& e)
{
	if(!fan1_set || !fan2_set || !quad_set)
		return;

	//update the map

	//update the path

	//send the path
}