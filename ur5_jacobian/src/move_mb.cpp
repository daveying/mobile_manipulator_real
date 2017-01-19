#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/Twist.h"

#include "hks_gripper/hksGripper.h"


void statusCallback(const std_msgs::Int8::ConstPtr& msg);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "yaskawa_demo");
	ros::NodeHandle n;
	
	ros::Publisher pub_agv = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);


	int step = -1;
	double run_time = 8;  //TODO

	geometry_msgs::Twist agv_msg;
	agv_msg.linear.x = 0.0;
	
	while(ros::ok())
	{
		if(step == -1)
		{
			agv_msg.linear.x -= 0.1;
			pub_agv.publish(agv_msg);
			ros::Duration(0.1).sleep();
			agv_msg.linear.x -= 0.1;
			pub_agv.publish(agv_msg);
			ros::Duration(0.1).sleep();
			agv_msg.linear.x -= 0.2;
			pub_agv.publish(agv_msg);
			ros::Duration(0.1).sleep();
			agv_msg.linear.x -= 0.2;
			pub_agv.publish(agv_msg);
			ros::Duration(0.1).sleep();
			agv_msg.linear.x -= 0.2;
			pub_agv.publish(agv_msg);
			ros::Duration(0.1).sleep();
			agv_msg.linear.x -= 0.2;
			pub_agv.publish(agv_msg);
			ros::Duration(0.1).sleep();
			agv_msg.linear.x -= 0.1;
			pub_agv.publish(agv_msg);
			//后退到最大速度
			ros::Duration(run_time).sleep();  //TODO
			
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
			//减速到0
			ros::Duration(0.5).sleep();
			
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
			//加速到最大速度
			ros::Duration(run_time).sleep();   //TODO
			
			agv_msg.linear.x -= 0.1;
			pub_agv.publish(agv_msg);
			ros::Duration(0.1).sleep();
			agv_msg.linear.x -= 0.1;
			pub_agv.publish(agv_msg);
			ros::Duration(0.1).sleep();
			agv_msg.linear.x -= 0.2;
			pub_agv.publish(agv_msg);
			ros::Duration(0.1).sleep();
			agv_msg.linear.x -= 0.2;
			pub_agv.publish(agv_msg);
			ros::Duration(0.1).sleep();
			agv_msg.linear.x -= 0.2;
			pub_agv.publish(agv_msg);
			ros::Duration(0.1).sleep();
			agv_msg.linear.x -= 0.2;
			pub_agv.publish(agv_msg);
			ros::Duration(0.1).sleep();
			agv_msg.linear.x -= 0.1;
			pub_agv.publish(agv_msg);
			//减速到0
			step = 0;
		}
		ros::spinOnce();
	}
	
	
}

