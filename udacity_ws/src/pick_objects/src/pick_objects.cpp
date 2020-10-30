#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pick_objects");

	MoveBaseClient ac("move_base", true);

	while(!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Wait for the move_base action server to come up");	
	}

	move_base_msgs::MoveBaseGoal goal_pickup, goal_dropoff;
	
	// Configure goal for pick up
	goal_pickup.target_pose.header.frame_id = "map";
	goal_pickup.target_pose.header.stamp = ros::Time::now();
	goal_pickup.target_pose.pose.position.x = 3.0;
	goal_pickup.target_pose.pose.position.y = 2.0;
	goal_pickup.target_pose.pose.orientation.w = 1.0;


	// Configure goal for drop off
	goal_dropoff.target_pose.header.frame_id = "map";
	goal_dropoff.target_pose.header.stamp = ros::Time::now();
	goal_dropoff.target_pose.pose.position.x = 11.0;
	goal_dropoff.target_pose.pose.position.y = 6.0;
	goal_dropoff.target_pose.pose.orientation.w = 1.0;

	ROS_INFO("Sending pick up goal to robot!");
	ac.sendGoal(goal_pickup);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Robot reached at the pick up goal!");
	else
		ROS_INFO("Robot failed to move to the pick up goal!");

	ros::Duration(5.0).sleep();

	ROS_INFO("Sending drop off goal to robot!");
	ac.sendGoal(goal_dropoff);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Robot reached at the drop off goal!");
	else
		ROS_INFO("Robot failed to move to the drop off goal!");

	return 0;
	
}
