#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

ros::ServiceClient client;

void drive_robot(float lin_x, float ang_z)
{
	ball_chaser::DriveToTarget srv;
	
	srv.request.linear_x = lin_x;
	srv.request.angular_z = ang_z;

	if (!client.call(srv))
		ROS_ERROR("Failed to call service drive_bot");	

}

void process_image_callback(const sensor_msgs::Image img)
{
	int white_pixel = 255;
	
	for (int i = 0; i < img.height * img.step; i++)
	{
		if (img.data[i] == white_pixel)
		{	
		
			ROS_INFO("Found white ball");

			if ((i % img.step) < (int)(img.step / 4))
				drive_robot(0.0, 0.05);
			else if ((i % img.step) > (int)(3 * img.step / 4))
				drive_robot(0.0, -0.05);
			else
				drive_robot(0.1, 0.0);

			break;
		}
		else
			drive_robot(0.0, 0.0);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "process_image");

	ros::NodeHandle n;

	client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

	ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

	ros::spin();

	return 0;
}
