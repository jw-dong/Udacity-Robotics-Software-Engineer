#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

float odom_x = 0.0, odom_y = 0.0;
float odom_x_offset = 2.0, odom_y_offset = 0.0;

void callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom_x = msg->pose.pose.position.x + odom_x_offset;
	odom_y = msg->pose.pose.position.y + odom_y_offset;
}

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "add_markers");
	ros::NodeHandle n;
	ros::Rate r(5);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_msgs", 1);
	ros::Subscriber odom_sub = n.subscribe("/odom", 1000, callback);

	//set initial shape type to be a cube
	uint32_t shape = visualization_msgs::Marker::CUBE;

	float tolerance = 0.15;
	float x_dist = 0.0, y_dist = 0.0;
	float x_dropoff = 11.0, y_dropoff = 6.0;
	bool picked = false;

	while(ros::ok())
	{
		visualization_msgs::Marker marker;
		
		//set frame ID and timestamp. 
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time::now();

		//set the namespace and id for this marker. This serves to create a unique ID
		//any marker sent with the same namespace and id will overwrite the old one
		marker.ns = "basic_shapes";
		marker.id = 0;

		//set the marker type. Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		marker.type = shape;

		//set the pose of the marker. This is full 6 DOF pose relative to the frame/time specified in the header
		marker.pose.position.x = 3.0;
		marker.pose.position.y = 2.0;
		marker.pose.position.z = 0.0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		//set the scale of the marker
		marker.scale.x = 0.25;
		marker.scale.y = 0.25;
		marker.scale.z = 0.25;

		//set the color, be sure to set alpha to something non-zero
		marker.color.r = 0.0f;
		marker.color.g = 0.0f;
		marker.color.b = 1.0f;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration();

		if (!picked)
		{	
			ROS_INFO("I'm here! %f, %f", odom_x, odom_y);

			x_dist = fabs(odom_x - marker.pose.position.x);	
			y_dist = fabs(odom_y - marker.pose.position.y);			
			if (x_dist < tolerance && y_dist < tolerance)
			{
				marker.action = visualization_msgs::Marker::DELETE;
				picked = true;
			}
			else
			{
				marker.action = visualization_msgs::Marker::ADD;
			}

			marker_pub.publish(marker);

		}
		else 
		{
			x_dist = fabs(odom_x - x_dropoff);	
			y_dist = fabs(odom_y - y_dropoff);

			if (x_dist < tolerance && y_dist < tolerance)
			{
				marker.pose.position.x = 11.0;
				marker.pose.position.y = 6.0;
				marker.pose.position.z = 0.0;
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;
				marker.action = visualization_msgs::Marker::ADD;
				marker_pub.publish(marker);
			}	
		}

		ros::spinOnce();	
	}

	r.sleep();
}
