#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

ros::Publisher motor_command_publisher;

bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
{
	geometry_msgs::Twist motor_command;

	motor_command.linear.x = req.linear_x;
	motor_command.angular.z = req.angular_z;

	motor_command_publisher.publish(motor_command);

	// Return a response message
	res.msg_feedback = "Wheel velocity has been set to: linear_x: " + std::to_string(motor_command.linear.x) + " and angular_z: " + std::to_string(motor_command.angular.z);
	ROS_INFO_STREAM(res.msg_feedback);

	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "drive_bot");

	ros::NodeHandle n;

	motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	//Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
	ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

	ros::spin();

	return 0;
}
