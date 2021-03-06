//sbpl planner wrapper
#include <sbpl_plan/SbplPlanner.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_utils/GeometryUtilsROS.h>
#include <vector>

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
	delete map;
	delete planner;
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
	//plann update rate
	rate = 0.5;

	//goal pos
	goal_pos = gu::Vec3(1.8,2.5,0); //need to change in matlab also

	//map parameter
	grid_size = 0.2;
	fan_R = 0.5;
	//build map
	map = new Grid2D(grid_size,fan_R);

	//search parameter
	bsearchuntilfirstsolution = false;
    bforwardsearch = true;
    initialEpsilon = 3.0;
    obsthresh = 1;
    allocated_time_secs = 500.0; // in seconds

    //build searcher
	environment_nav2D.InitializeEnv(map->x_width, map->y_width, map->getMap(), 0, 0, 0, 0, obsthresh);
	environment_nav2D.InitializeMDPCfg(&MDPCfg);
    planner = new ADPlanner(&environment_nav2D, bforwardsearch);
	planner->set_search_mode(bsearchuntilfirstsolution);
	planner->set_initialsolution_eps(initialEpsilon);

	return true;
}

bool SbplPlanner::registerCallbacks(const ros::NodeHandle& n)
{
	ros::NodeHandle ln(n);
	fan1_sub = ln.subscribe("fan1/odom",10,&SbplPlanner::fan1Callback, this);
	fan2_sub = ln.subscribe("fan2/odom",10,&SbplPlanner::fan2Callback, this);
	quad_sub = ln.subscribe("odom",10,&SbplPlanner::quadCallback, this);
	plan_pub = ln.advertise<geometry_msgs::PoseArray>("plan_path",10,false);
	plan_timer = ln.createTimer(rate, &SbplPlanner::planTimerCallback, this);

	return true;
}

void SbplPlanner::fan1Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	fan1_pos = gr::fromROS(msg->pose.pose.position);
	//ROS_INFO_STREAM("fan1"<<fan1_pos.x()<<","<<fan1_pos.y());
	fan1_set = true;
}

void SbplPlanner::fan2Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	fan2_pos = gr::fromROS(msg->pose.pose.position);
	//ROS_INFO_STREAM("fan2"<<fan2_pos.x()<<","<<fan2_pos.y());
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
	map->makeGrid2D(fan1_pos, fan2_pos, false);

	//update the environment
	std::vector<int> solution_stateIDs_V;
	int bRet = 0;
	gu::Vec3 start = map->findGrids(quad_pos);
	gu::Vec3 end = map->findGrids(goal_pos);
	ROS_INFO_STREAM("start:"<<start.x()<<","<<start.y()<<"end:"<<end.x()<<","<<end.y());
	environment_nav2D.SetConfiguration(map->x_width, map->y_width, map->getMap()
		  , (int)(start.x()), (int)(start.y()), (int)(end.x()), (int)(end.y()));
	environment_nav2D.SetStart((int)(start.x()), (int)(start.y()));
	environment_nav2D.SetGoal((int)(end.x()), (int)(end.y()));
	environment_nav2D.InitializeMDPCfg(&MDPCfg);
	//ROS_INFO_STREAM("startid:"<<MDPCfg.startstateid<<"endid:"<<MDPCfg.goalstateid);
	planner->set_start(MDPCfg.startstateid);
	planner->set_goal(MDPCfg.goalstateid);

	//recompute path
	bRet = planner->replan(allocated_time_secs, &solution_stateIDs_V);
	ROS_INFO_STREAM("path size: "<<solution_stateIDs_V.size());

	//create the path message
	geometry_msgs::PoseArray path_msg;
	path_msg.header.stamp = e.current_real;
	path_msg.poses.resize(solution_stateIDs_V.size());
	int x,y;
	for(int i=0; i<solution_stateIDs_V.size(); i++)
	{
		environment_nav2D.GetCoordFromState(solution_stateIDs_V[i], x, y);
		path_msg.poses[i].position.x = map->findCellCenterCoord(x,'X');
		path_msg.poses[i].position.y = map->findCellCenterCoord(y,'Y');
	}

	//publish the path message
	plan_pub.publish(path_msg);
}

