#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv)
{
	bool first = true;
	ros::init(argc, argv, "joint_speed_talker");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<trajectory_msgs::JointTrajectory>("/ur_driver/joint_speed", 1000);

	ros::Rate loop_rate(100);
	sleep(0.5);
	trajectory_msgs::JointTrajectory trj;
	trajectory_msgs::JointTrajectoryPoint trjp;
	trjp.velocities.push_back(0.0);
	trjp.velocities.push_back(0.0);
	trjp.velocities.push_back(0.0);
	trjp.velocities.push_back(0.0);
	trjp.velocities.push_back(0.0);
	trjp.velocities.push_back(0.0);
	trj.points.push_back(trjp);
	ros::Time be = ros::Time::now();
	while(ros::ok())
	{
		trj.header.stamp = ros::Time::now();
		ros::Time k = ros::Time::now();
		if(first)
		{
			first = false;
			be = k;
		}
		ros::Duration du = k - be;
		std_msgs::Float64 vel;
		vel.data = 0;
		vel.data = 0.5*sin(du.toSec());
		trj.points[0].velocities[0] = vel.data;

		//vel.data = 0.15*sin(du.toSec());
		//trj.points[0].velocities[1] = vel.data;

		//vel.data = 0.25*sin(du.toSec());
		//trj.points[0].velocities[2] = vel.data;

		//vel.data = 0.35*sin(du.toSec());
		//trj.points[0].velocities[3] = vel.data;

		//vel.data = 0.15*sin(du.toSec());
		//trj.points[0].velocities[4] = vel.data;

		//vel.data = 0.15*sin(du.toSec());
		//trj.points[0].velocities[5] = vel.data;
		chatter_pub.publish(trj);
		if(du.toSec() > 18.8495556*10)break;
		ros::spinOnce();
		loop_rate.sleep();
	}
	loop_rate.sleep();
	trj.points[0].velocities[0] = 0;
	trj.header.stamp = ros::Time::now();
	chatter_pub.publish(trj);
}
