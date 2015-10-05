#include <simple_service/MoveToSimpleAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseArray.h>
#include <signal.h>
#include <iostream>


class WayPointsFollower {

	bool waypoints_changed_;
	ros::NodeHandle nh_;
	ros::Subscriber wayPointsSubscriber;
	geometry_msgs::PoseArrayConstPtr pose_array_;
	actionlib::SimpleActionClient <simple_service::MoveToSimpleAction> client_;

public:
	WayPointsFollower();
	void wayPointsCallback(const geometry_msgs::PoseArrayConstPtr& pose_array);
	void reachTheGoal();
};

WayPointsFollower::WayPointsFollower():
	client_ ("move_to_simple", true) {
	this->wayPointsSubscriber = nh_.subscribe("pose_array_topic", 1, &WayPointsFollower::wayPointsCallback, this);
	ROS_INFO("We await the server...");
	client_.waitForServer();
	ROS_INFO("Server is here. Everything's under control.");
	waypoints_changed_ = false;
}

void WayPointsFollower::wayPointsCallback(const geometry_msgs::PoseArrayConstPtr& pose_array) {
	this->pose_array_ = pose_array;
	waypoints_changed_ = true;
}

void WayPointsFollower::reachTheGoal() {
	while(true && ros::ok()) {

		while(!pose_array_) {
			ROS_INFO("Go ahead and gimme a goal!");
			ros::spinOnce();
			sleep(1);
		}

		this->waypoints_changed_ = false;

		ROS_INFO("Got a set of way-points to follow.");

		for(int i = 0; i < pose_array_->poses.size(); i++) {	
			simple_service::MoveToSimpleGoal A;
			A.goal_pose.pose = pose_array_->poses[i];
			A.goal_pose.header.frame_id = "robot_0/odom";
			A.driving_direction = simple_service::MoveToSimpleGoal::FORWARD;
			A.xy_tolerance = 1.0;
			A.yaw_tolerance = 3.2;

			client_.sendGoal(A);
			ROS_INFO("Despatched Way-Point No. %d", i);
			client_.waitForResult();
			ROS_INFO("No. %d:=> Success.", i);

			ros::spinOnce();
			if(this->waypoints_changed_) {
				i = -1;
				ROS_INFO("Updating waypoints...");
				waypoints_changed_ = false;
			}
		}

		pose_array_.reset();
	}
}

int main(int argn, char *args[])
{
	ros::init(argn, args, "client_simple");

	WayPointsFollower wpf;
	wpf.reachTheGoal();
	return 0;
}
