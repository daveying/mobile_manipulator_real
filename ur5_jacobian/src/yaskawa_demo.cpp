#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/Twist.h"

#include "hks_gripper/hksGripper.h"

int count = 0;
bool beginn = false;

void statusCallback(const std_msgs::Int8::ConstPtr& msg);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "yaskawa_demo");
	ros::NodeHandle n;
	
	ros::Subscriber sub_status = n.subscribe("/visp_auto_tracker/status", 1000, statusCallback);
	ros::Publisher pub_dist = n.advertise<std_msgs::Float64>("/goal_distance", 1000);
	ros::Publisher pub_control = n.advertise<std_msgs::Int8>("/jacobian_ctrl", 1000);
	ros::Publisher pub_agv = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	ros::ServiceClient client_gripper = n.serviceClient<hks_gripper::hksGripper>("hksGpripperSrv2");

	hks_gripper::hksGripper srv;
	srv.request.position = 80;
	if (client_gripper.call(srv))
	{
		ROS_INFO("Result: %ld", (long int)srv.response.status);
	}
	else
	{
		ROS_ERROR("Failed to call service add_two_ints");
	}

	int step = -1;

	geometry_msgs::Twist agv_msg;
	agv_msg.linear.x = 0.0;
	
	while(ros::ok())
	{
		if(step == -1)
		{
			agv_msg.linear.x += 0.1;
			pub_agv.publish(agv_msg);
			ros::Duration(0.1).sleep();
			agv_msg.linear.x += 0.1;
			pub_agv.publish(agv_msg);
			ros::Duration(0.1).sleep();
			agv_msg.linear.x += 0.2;
			pub_agv.publish(agv_msg);
			ros::Duration(0.1).sleep();
			agv_msg.linear.x += 0.2;
			pub_agv.publish(agv_msg);
			ros::Duration(0.1).sleep();
			agv_msg.linear.x += 0.2;
			pub_agv.publish(agv_msg);
			ros::Duration(0.1).sleep();
			agv_msg.linear.x += 0.2;
			pub_agv.publish(agv_msg);
			ros::Duration(0.1).sleep();
			agv_msg.linear.x += 0.1;
			pub_agv.publish(agv_msg);
			step = 0;
		}
		std_msgs::Float64 dist_msg;
		if(step == 1)
		{
			dist_msg.data = 0.42;
			pub_dist.publish(dist_msg);
			ros::Duration(2.5).sleep();
		}

		if(step == 2)
		{
			dist_msg.data = 0.3;
			pub_dist.publish(dist_msg);
			ros::Duration(1.5).sleep();
			srv.request.position = 110;
			if (client_gripper.call(srv))
			{
				ROS_INFO("Result: %ld", (long int)srv.response.status);
			}
			else
			{
				ROS_ERROR("Failed to call service add_two_ints");
			}
			ros::Duration(2).sleep();
		}

		if(step == 3)
		{
			dist_msg.data = 0.42;
			pub_dist.publish(dist_msg);
			ros::Duration(1.5).sleep();
		}

		if(step == 4)
		{
			std_msgs::Int8 ctrl_msg;
			ctrl_msg.data = 0;
			pub_control.publish(ctrl_msg);
			ros::Duration(5).sleep();
			
		}
		if(step == 5)
		{
			agv_msg.linear.x -= 0.1;
			ros::Duration(0.5).sleep();
			pub_agv.publish(agv_msg);
			agv_msg.linear.x -= 0.2;
			ros::Duration(0.5).sleep();
			pub_agv.publish(agv_msg);
			agv_msg.linear.x -= 0.2;
			ros::Duration(0.5).sleep();
			pub_agv.publish(agv_msg);
			agv_msg.linear.x = 0;
			ros::Duration(0.5).sleep();
			pub_agv.publish(agv_msg);
		}
		if(beginn)step++;
		if(step == 200)step = 200;
		ros::spinOnce();
	}
	
	
}

void statusCallback(const std_msgs::Int8::ConstPtr& msg)
{
	if(msg->data == 3)
	{
		count++;
	}
	else
	{
		count = 0;
	}
	if(count>=6)
	{
		count = 6;
		beginn = true;
	}
	ROS_INFO("count: %d", count);
}
