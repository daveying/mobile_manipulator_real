#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "goal_tf_broadcaster");
	ros::NodeHandle n;
	tf::TransformBroadcaster br;
	tf::TransformListener listener;
	tf::Transform transform;
	tf::StampedTransform stransform;
	bool s = false;

	while(!s && ros::ok())
	{
		try
		{
			listener.lookupTransform("world", "tool0", ros::Time(0), stransform);
			s = true;
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1).sleep();
			continue; 
		}
	}
	transform = stransform;
	ros::Rate rate(20);
	while(ros::ok())
	{
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "goal"));
		rate.sleep();
	}
	ros::spin();
	return 0;

}
