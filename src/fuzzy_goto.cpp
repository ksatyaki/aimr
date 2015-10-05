#include <aimr/fuzzy.h>
#include <aimr/fuzzy_velocity.h>

enum RequiredHeading { FULLRIGHT = 0, JUSTRIGHT, AHEAD, JUSTLEFT, FULLLEFT };
enum RequiredSpeed { BACK = 0, STOP, SLOW, FAST };

#include <simple_service/MoveToSimpleAction.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>

#include <string>
#include <cmath>

#include <iostream>

#define GOAL_TOLERANCE 0.1

namespace fuzzy_service {

class MoveToFuzzyServer
{
	/**
	 * A NodeHandle for class' access.
	 */
	ros::NodeHandle _nh;

	/**
	 * The actual actionlib server.
	 */
	actionlib::SimpleActionServer <simple_service::MoveToSimpleAction> _server;

	/**
	 * A Listener to read transform information.
	 */
	tf::TransformListener _tf_listener;

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
	 * Called when goal is received.
	 */
	void goalCallback(const simple_service::MoveToSimpleGoalConstPtr& goal);

	/**
	 * Get the current pose.
	 */
	geometry_msgs::PoseStamped getCurrentPose();

	/**
	 * Constructor takes the topic name for command_velocity topic as a std::string.
	 */
	MoveToFuzzyServer(std::string cmd_vel_topic_name = "cmd_vel");

	virtual ~MoveToFuzzyServer();

};

void MoveToFuzzyServer::laserCallback(const sensor_msgs::LaserScanConstPtr& laser_scan)
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

//
// Should be actual c++ file.
//
MoveToFuzzyServer::MoveToFuzzyServer (std::string cmd_vel_topic_name) :
_server(_nh, "move_to_simple", boost::bind(&MoveToFuzzyServer::goalCallback, this, _1), false) ,
_tf_listener(ros::Duration(2))
{
	_cmd_vel_pub = _nh.advertise <geometry_msgs::Twist> (cmd_vel_topic_name, 1);
	_laser_sub = _nh.subscribe("scan", 1, &MoveToFuzzyServer::laserCallback, this);

	ROS_INFO("Starting server...");
	_server.start();
	ROS_INFO("Server is up and running.");
}


MoveToFuzzyServer::~MoveToFuzzyServer()
{

}

void MoveToFuzzyServer::goalCallback(const simple_service::MoveToSimpleGoalConstPtr& goal)
{
	ROS_INFO("Got a new goal to work on... Hmmm...");

	int driving_direction = 1;

	if(goal->driving_direction == simple_service::MoveToSimpleGoal::REVERSE)
		driving_direction = -1;

	geometry_msgs::PoseStamped start_pose = getCurrentPose();

	geometry_msgs::PoseStamped current_pose;
	current_pose.header = start_pose.header;
	current_pose.pose = start_pose.pose;

	geometry_msgs::Point final_point;
	float angle;

	tf::Vector3 pose_in_base (goal->goal_pose.pose.position.x, goal->goal_pose.pose.position.y, goal->goal_pose.pose.position.z);
	tf::Vector3 pose_in_odom;

	tf::StampedTransform base_to_odom;

	_tf_listener.waitForTransform("robot_0/odom", goal->goal_pose.header.frame_id, ros::Time(0), ros::Duration(1.0));

	try{
		_tf_listener.lookupTransform("robot_0/odom", goal->goal_pose.header.frame_id, ros::Time(0), base_to_odom);
	}
	catch(tf::TransformException& ex)
	{
		ROS_INFO("COCKED!");
	}

	pose_in_odom = base_to_odom * pose_in_base;

	final_point.x = pose_in_odom.x();
	final_point.y = pose_in_odom.y();

	ros::Rate publish_rate(10);

	//ROS_INFO("Final point to reach is %lf, %lf.", final_point.x, final_point.y);

	float xy_tolerance = goal->xy_tolerance > 0.075 ? goal->xy_tolerance : 0.075;

	geometry_msgs::Twist resultant_velocity;
	FuzzyVelocity fv(4, 5);

	// Predicates for distance to goal.
	fuzzy::Predicate FAR_OFF, NEAR, AT;

	// Predicates for heading to goal.
	fuzzy::Predicate LLEFT, LEFT, STRAIGHT, RIGHT, RRIGHT;

	// Predicates for obstacle position.
	fuzzy::Predicate OBS_LLEFT, OBS_LEFT, OBS_STRAIGHT, OBS_RIGHT, OBS_RRIGHT;

	while(!equals(final_point, current_pose.pose.position, xy_tolerance))
	{
		////////////////////////////
		///// Publish velocity /////
		////////////////////////////
		if(driving_direction == 1)
		{
			angle = atan2(final_point.y - current_pose.pose.position.y, final_point.x - current_pose.pose.position.x);
		}
		else
			angle = atan2(current_pose.pose.position.y - final_point.y, current_pose.pose.position.x - final_point.x);

		float error_fi = angle - (2 * atan2(current_pose.pose.orientation.z, current_pose.pose.orientation.w));
		error_fi = atan2(sin(error_fi),cos(error_fi));

		float error_dist = sqrt( (final_point.x - current_pose.pose.position.x) * (final_point.x - current_pose.pose.position.x) +
				   (final_point.y - current_pose.pose.position.y) * (final_point.y - current_pose.pose.position.y));


		fv.resetFuzzySets();

		// Compute the predicates.
		FAR_OFF = fuzzy::RampUp(error_dist, 2.5, 3.0);
		NEAR = fuzzy::AND(fuzzy::RampUp(error_dist, 0.1, 1.0), fuzzy::RampDown(error_dist, 2.5, 3.0));
		AT = fuzzy::RampDown(error_dist, 0.0, 0.1);

		LLEFT = fuzzy::RampUp(error_fi, 1.00, 1.5);
		LEFT = fuzzy::AND(fuzzy::RampUp(error_fi, 0.0, 0.05), fuzzy::RampDown(error_fi, 1.00, 1.5));
		STRAIGHT = fuzzy::AND(fuzzy::RampUp(error_fi, -0.05, 0.0) , fuzzy::RampDown(error_fi, 0.0, 0.05));
		RIGHT = fuzzy::AND(fuzzy::RampUp(error_fi, -2.5, -1.00), fuzzy::RampDown(error_fi, -0.05, 0.0));
		RRIGHT = fuzzy::RampDown(error_fi, -1.5, -1.00);

		const float danger_dist = 0.5;
		const float safe_dist = 1.0;

		// Obstacle avoidance predicates.
		OBS_RRIGHT = fuzzy::RampDown( _obstacle_distance[0], danger_dist, safe_dist);
		OBS_RIGHT = fuzzy::RampDown(_obstacle_distance[1], danger_dist, safe_dist);
		OBS_STRAIGHT = fuzzy::RampDown(_obstacle_distance[2], danger_dist, safe_dist);
		OBS_LEFT = fuzzy::RampDown(_obstacle_distance[3], danger_dist, safe_dist);
		OBS_LLEFT = fuzzy::RampDown( _obstacle_distance[4], danger_dist, safe_dist);

		printf("\nOBS_LEFT = %lf", OBS_LLEFT);
		printf("\nOBS_STRAIGHT = %lf", OBS_STRAIGHT);
		printf("\nOBS_RIGHT = %lf", OBS_RRIGHT);

		// Lets do the behaviour now!
		fuzzy::IF(fuzzy::AND(LLEFT, fuzzy::NOT(OBS_LLEFT))); fv.setRotationalVelocity(FULLLEFT);

		fuzzy::IF(fuzzy::AND(LEFT, fuzzy::NOT(OBS_LEFT))); fv.setRotationalVelocity(JUSTLEFT);
		fuzzy::IF(fuzzy::AND(STRAIGHT, fuzzy::NOT(OBS_STRAIGHT))); fv.setRotationalVelocity(AHEAD);
		fuzzy::IF(fuzzy::AND(RIGHT, fuzzy::NOT(OBS_RIGHT))); fv.setRotationalVelocity(JUSTRIGHT);
		fuzzy::IF(fuzzy::AND(RRIGHT, fuzzy::NOT(OBS_RRIGHT))); fv.setRotationalVelocity(FULLRIGHT);

//		fuzzy::IF(FAR_OFF); fv.setLinearVelocity(FAST);
		fuzzy::IF(NEAR); fv.setLinearVelocity(SLOW);
		fuzzy::IF(AT); fv.setLinearVelocity(STOP);

//		printf("\n************************");
//		printf("\nBEFORE THE OBS AVOIDANCE");
//		printf("\n************************\n");
//		fv.printSets();

		// Fuzzy activations.
		fuzzy::IF( fuzzy::AND(OBS_LEFT, fuzzy::NOT(OBS_RRIGHT)) ); fv.setRotationalVelocity(FULLRIGHT); fv.setLinearVelocity(STOP);
		fuzzy::IF( fuzzy::AND(OBS_RIGHT, fuzzy::NOT(OBS_LLEFT)) ); fv.setRotationalVelocity(FULLLEFT); fv.setLinearVelocity(STOP);
		fuzzy::IF( fuzzy::AND(OBS_LLEFT, fuzzy::NOT(OBS_RIGHT)) ); fv.setRotationalVelocity(JUSTRIGHT); fv.setLinearVelocity(SLOW);
		fuzzy::IF( fuzzy::AND(OBS_RRIGHT, fuzzy::NOT(OBS_LEFT)) ); fv.setRotationalVelocity(JUSTLEFT); fv.setLinearVelocity(SLOW);

		fuzzy::IF( OBS_STRAIGHT ); fv.setLinearVelocity(SLOW);
		fuzzy::IF( fuzzy::NOT( fuzzy::OR(OBS_RIGHT, fuzzy::OR(OBS_LEFT, fuzzy::OR(OBS_RRIGHT, fuzzy::OR(OBS_STRAIGHT, OBS_LLEFT)))))) ; fv.setLinearVelocity(FAST);
		fuzzy::IF( fuzzy::AND( fuzzy::AND(OBS_STRAIGHT, OBS_LLEFT), OBS_RRIGHT) ); fv.setLinearVelocity(BACK);

		fv.deFuzzifyVelocity();

		printf("\n***********************");
		printf("\nAFTER THE OBS AVOIDANCE");
		printf("\n***********************\n");
		fv.printSets();
		resultant_velocity.linear.x = 0.3*fv.getLinear();
		resultant_velocity.angular.z = fv.getRotational();

		ROS_INFO("LINEAR: %lf, ANGULAR: %lf", resultant_velocity.linear.x, resultant_velocity.angular.z);

		_cmd_vel_pub.publish(resultant_velocity);
		//ROS_INFO("Final point to reach is %lf, %lf.", final_point.x, final_point.y);
		//ROS_INFO("Velocities: x, theta = (%f, %f)", resultant_velocity.linear.x, resultant_velocity.angular.z);

		//ROS_INFO("ANGLE: (%f) and [%f]", error_fi, angle);

		////////////////////////////////////
		///// Recalculate our position /////
		////////////////////////////////////

		current_pose = getCurrentPose();

		publish_rate.sleep();
	}

	// Execute the rotation behaviour.
	float goal_fi_relative = 2*atan2(goal->goal_pose.pose.orientation.z, goal->goal_pose.pose.orientation.w);
	float goal_fi = 2*atan2(start_pose.pose.orientation.z, start_pose.pose.orientation.w) + goal_fi_relative;
	goal_fi = atan2(sin(goal_fi), cos(goal_fi));

	float our_fi = 2*atan2(current_pose.pose.orientation.z, current_pose.pose.orientation.w);

	float yaw_tolerance = goal->yaw_tolerance > 0.15 ? goal->yaw_tolerance : 0.15;;

	while(ros::ok())
	{
		current_pose = getCurrentPose();

		our_fi = 2*atan2(current_pose.pose.orientation.z, current_pose.pose.orientation.w);

		float del_fi = atan2(sin(goal_fi - our_fi), cos((goal_fi - our_fi)));

		geometry_msgs::Twist resultant_velocity;

		resultant_velocity.angular.z = (del_fi * 0.66);

		if(resultant_velocity.angular.z > 1.5)
			resultant_velocity.angular.z = 1.5;
		else if(resultant_velocity.angular.z < -1.5)
			resultant_velocity.angular.z = -1.5;

		_cmd_vel_pub.publish(resultant_velocity);

		//ROS_INFO("ANGLE: goal_relative: %f", goal_fi_relative);
		//ROS_INFO("del_fi: (%f)", del_fi);

		publish_rate.sleep();

		if(fabs(del_fi) < yaw_tolerance )
			break;
	}

	ROS_INFO("Action succeeded!");
	_server.setSucceeded();

}

geometry_msgs::PoseStamped MoveToFuzzyServer::getCurrentPose()
{
	ros::Time _now_stamp_ = ros::Time::now();

	tf::StampedTransform start_pose_in_tf;

	_tf_listener.waitForTransform("robot_0/odom", "robot_0/base_link", _now_stamp_, ros::Duration(2.0));
	try
	{
		_tf_listener.lookupTransform("robot_0/odom", "robot_0/base_link", _now_stamp_, start_pose_in_tf);
	}
	catch(tf::TransformException& ex)
	{
		ROS_INFO("TRANSFORMS ARE COCKED-UP PAL! Why is that :=> %s", ex.what());
	}

	tf::Vector3 start_position = start_pose_in_tf.getOrigin();
	tf::Quaternion start_orientation = start_pose_in_tf.getRotation();

	geometry_msgs::PoseStamped start_pose;
	start_pose.header.stamp = start_pose_in_tf.stamp_;
	start_pose.header.frame_id = start_pose_in_tf.frame_id_;

	tf::pointTFToMsg(start_position, start_pose.pose.position);
	tf::quaternionTFToMsg(start_orientation, start_pose.pose.orientation);

	return start_pose;
}

bool MoveToFuzzyServer::equals(geometry_msgs::Point a, geometry_msgs::Point b, float tolerance)
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

	fuzzy_service::MoveToFuzzyServer move_to_server;

	ros::spin();
}


