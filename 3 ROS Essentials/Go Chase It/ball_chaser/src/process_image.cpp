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
	int direction = 0; //0 stop, 1 left, 2 straight, 3 right
	
	for (int i = 0; i < img.height * img.width; i++)
	{      
		if (img.data[3*i] == white_pixel && img.data[3*i+1] == white_pixel && img.data[3*i+2] == white_pixel)
		{	
			if ((i % img.width) < (int)(img.width / 4))
				direction = 1;
			else if ((i % img.width) > (int)(3 * img.width / 4))
				direction = 3;
			else
				direction = 2;

			break;
		}
	}

	switch (direction)
	{
		case 1:
			drive_robot(0.0, 0.2);
			break;
		case 2:
			drive_robot(0.2, 0.0);
			break;
		case 3:
			drive_robot(0.0, -0.2);
			break;
		default:
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
