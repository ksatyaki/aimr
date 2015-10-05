#include <aimr/fuzzy.h>
#include <aimr/fuzzy_velocity.h>

enum RequiredHeading { FULLRIGHT = 0, JUSTRIGHT, AHEAD, JUSTLEFT, FULLLEFT };
enum RequiredSpeed { BACK = 0, STOP, SLOW, FAST };

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_listener.h>

#include <string>
#include <cmath>

#include <iostream>

#define GOAL_TOLERANCE 0.1

namespace fuzzy_service {

class FuzzyWallFollow
{
	/**
	 * A NodeHandle for class' access.
	 */
	ros::NodeHandle _nh;

	/**
	 * A ros Publisher to publish velocity messages.
	 */
	ros::Publisher _cmd_vel_pub;

	/**
	 * To equate two poses based on a tolerance.
	 */
	bool equals(geometry_msgs::Point a, geometry_msgs::Point b, float tolerance = GOAL_TOLERANCE);

	/**
	 * Subscriber to the laser message.
	 */
	ros::Subscriber _laser_sub;

	/**
	 * Callback for the laser.
	 */
	void laserCallback(const sensor_msgs::LaserScanConstPtr& laser_scan);

	/**
	 * An array that holds the minimum obstacle distance values for the 5 regions.
	 */
	double _obstacle_distance[5];

public:
	/**
	 * Do the thing now.
	 */
	void do_wall_follow();

	/**
	 * Constructor takes the topic name for command_velocity topic as a std::string.
	 */
	FuzzyWallFollow(std::string cmd_vel_topic_name = "cmd_vel");

	virtual ~FuzzyWallFollow();

};

void FuzzyWallFollow::laserCallback(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
	const int indices[] = { 0, 18, 72, 108, 144, 180 };


	for(int j = 0; j < 5; j++)
	{
		float min = 5.0;

		for(int i = indices[j]; i < indices[j+1]; i++)
		{
			if(laser_scan->ranges[i] < laser_scan->range_max && laser_scan->ranges[i] > laser_scan->range_min)
			{
				if(laser_scan->ranges[i] < min)
					min = laser_scan->ranges[i];
			}
		}

		_obstacle_distance[j] = min;
		//printf("\nObstacle Distance %d: %lf\n", j, _obstacle_distance[j]);
	}
}


FuzzyWallFollow::FuzzyWallFollow (std::string cmd_vel_topic_name)
{
	_cmd_vel_pub = _nh.advertise <geometry_msgs::Twist> (cmd_vel_topic_name, 1);
	_laser_sub = _nh.subscribe("scan", 1, &FuzzyWallFollow::laserCallback, this);

	ROS_INFO("Starting the thing now...");
}


FuzzyWallFollow::~FuzzyWallFollow()
{

}

void FuzzyWallFollow::do_wall_follow()
{
	ROS_INFO("Got a new goal to work on... Hmmm...");

	ros::Rate publish_rate(10);

	geometry_msgs::Twist resultant_velocity;
	FuzzyVelocity fv(4, 5);

	// Predicates for obstacle position.
	fuzzy::Predicate OBS_LLEFT, OBS_LEFT, OBS_STRAIGHT, OBS_RIGHT, OBS_RRIGHT;

	while(true && ros::ok())
	{
		ros::spinOnce();
		////////////////////////////
		///// Publish velocity /////
		////////////////////////////

		fv.resetFuzzySets();

		const float danger_dist = 0.5;
		const float safe_dist = 1.0;

		// Obstacle avoidance predicates.
		OBS_RRIGHT = fuzzy::RampDown( _obstacle_distance[0], danger_dist, safe_dist);
		OBS_RIGHT = fuzzy::RampDown(_obstacle_distance[1], danger_dist, safe_dist);
		OBS_STRAIGHT = fuzzy::RampDown(_obstacle_distance[2], danger_dist, safe_dist);
		OBS_LEFT = fuzzy::RampDown(_obstacle_distance[3], danger_dist, safe_dist);
		OBS_LLEFT = fuzzy::RampDown( _obstacle_distance[4], danger_dist, safe_dist);

		printf("\nOBS_LLEFT = %lf", OBS_LLEFT);
		printf("\nOBS_LEFT = %lf", OBS_LEFT);
		printf("\nOBS_STRAIGHT = %lf", OBS_STRAIGHT);
		printf("\nOBS_RIGHT = %lf", OBS_RIGHT);
		printf("\nOBS_RRIGHT = %lf\n", OBS_RRIGHT);

		// Lets do the behaviour now!
		fuzzy::IF(fuzzy::AND(OBS_LLEFT, OBS_LEFT)); fv.setRotationalVelocity(AHEAD);
		fuzzy::IF(fuzzy::AND(OBS_RRIGHT, OBS_RIGHT)); fv.setRotationalVelocity(AHEAD);
		
		fuzzy::IF( fuzzy::AND(fuzzy::OR( fuzzy::OR(OBS_STRAIGHT, OBS_RRIGHT), OBS_RIGHT), fuzzy::AND(OBS_LEFT, OBS_LLEFT) )); fv.setRotationalVelocity(FULLRIGHT);
		//fuzzy::IF(fuzzy::AND(fuzzy::OR (fuzzy::OR(OBS_STRAIGHT, OBS_RRIGHT), OBS_RIGHT), fuzzy::AND(fuzzy::NOT(OBS_LEFT), fuzzy::NOT(OBS_LLEFT)))); fv.setRotationalVelocity(FULLLEFT);
		fuzzy::IF(fuzzy::AND(OBS_STRAIGHT, fuzzy::NOT(OBS_RRIGHT))); fv.setRotationalVelocity(FULLRIGHT);
		fuzzy::IF(fuzzy::AND(OBS_STRAIGHT, fuzzy::NOT(OBS_LLEFT))); fv.setRotationalVelocity(FULLLEFT);
		fuzzy::IF(fuzzy::AND(OBS_LLEFT, fuzzy::NOT(OBS_LEFT))); fv.setRotationalVelocity(FULLLEFT);
		fuzzy::IF(fuzzy::AND(OBS_RRIGHT, fuzzy::NOT(OBS_RIGHT))); fv.setRotationalVelocity(FULLRIGHT);

		fuzzy::IF( fuzzy::AND(OBS_RIGHT, fuzzy::AND(OBS_LEFT, OBS_STRAIGHT))) ; fv.setLinearVelocity(STOP);

		fuzzy::IF( fuzzy::NOT(fuzzy::OR(OBS_RIGHT, fuzzy::OR(OBS_LEFT, fuzzy::OR(OBS_RRIGHT, fuzzy::OR(OBS_STRAIGHT, OBS_LLEFT)))))) ; fv.setLinearVelocity(FAST);

		fuzzy::IF( fuzzy::AND(fuzzy::AND(OBS_LLEFT, OBS_LEFT), fuzzy::NOT(OBS_STRAIGHT))); fv.setLinearVelocity(FAST);
		fuzzy::IF( fuzzy::AND(fuzzy::AND(OBS_RRIGHT, OBS_RIGHT), fuzzy::NOT(OBS_STRAIGHT))); fv.setLinearVelocity(FAST);
		
	   	fuzzy::IF( OBS_STRAIGHT ); fv.setLinearVelocity(SLOW);
		fuzzy::IF( fuzzy::AND( fuzzy::AND(OBS_STRAIGHT, OBS_LLEFT), OBS_RRIGHT) ); fv.setLinearVelocity(BACK);

		fv.deFuzzifyVelocity();
		fv.printSets();
		
		resultant_velocity.linear.x = 0.3*fv.getLinear();
		resultant_velocity.angular.z = fv.getRotational();

		ROS_INFO("LINEAR: %lf, ANGULAR: %lf", resultant_velocity.linear.x, resultant_velocity.angular.z);

		_cmd_vel_pub.publish(resultant_velocity);
		publish_rate.sleep();
	}
}

bool FuzzyWallFollow::equals(geometry_msgs::Point a, geometry_msgs::Point b, float tolerance)
{
	if(fabs(a.x - b.x) < tolerance &&
		fabs(a.y - b.y) < tolerance)
	return true;

	else return false;

}

}
geometry_msgs::Point operator + (geometry_msgs::Point a, geometry_msgs::Point b)
{
	geometry_msgs::Point result;

	result.x = a.x + b.x;
	result.y = a.y + b.y;
	result.z = a.z + b.z;

	return result;
}

int main(int argn, char* args[])
{
	ros::init(argn, args, "move_to_simple");

	fuzzy_service::FuzzyWallFollow wall_follower;

	wall_follower.do_wall_follow();

	return 0;
}


