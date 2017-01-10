#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>

tf::Transform camera_qrcode_transform;
ros::Time stamp;
std_msgs::Int8 status;
//tf::TransformListener* plistener;
int count = 0;

void qrPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void statusCallback(const std_msgs::Int8::ConstPtr& msg);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "qrcode_track_goal_broadercaster");
	ros::NodeHandle n;
	tf::TransformBroadcaster br;
	tf::TransformListener listener;
	//plistener = &listener;
	status.data = 0;
	
	ros::Subscriber sub_qr_pose = n.subscribe("/visp_auto_tracker/object_position", 1000, qrPoseCallback);
	ros::Subscriber sub_status = n.subscribe("/visp_auto_tracker/status", 1000, statusCallback);

	tf::Transform tool_camera_transform;
	tf::Transform qr_goal_transform;
	ros::Rate rate(10);
	
	qr_goal_transform.setOrigin(tf::Vector3(0, 0.064, 0.35));
	qr_goal_transform.setRotation(tf::Quaternion(1 , 0, 0, 0));
	camera_qrcode_transform.setOrigin(tf::Vector3(0,0,0));
	camera_qrcode_transform.setRotation(tf::Quaternion(0, 0, 0, 1));
	tool_camera_transform.setOrigin(tf::Vector3(0.064, -0.017, 0.045));
	tool_camera_transform.setRotation(tf::Quaternion(0, 0, -0.707106781, 0.707106781));
	tf::StampedTransform world_qr_stransform;
	tf::Transform world_goal_transform;

	
	while(ros::ok())
	{
		rate.sleep();
		ros::spinOnce();

		br.sendTransform(tf::StampedTransform(tool_camera_transform, ros::Time::now(), "tool0", "camera"));
		br.sendTransform(tf::StampedTransform(camera_qrcode_transform, stamp, "camera", "qr_code"));
		//br.sendTransform(tf::StampedTransform(qr_goal_transform, ros::Time::now(), "qr_code", "goal"));
		try
		{
			listener.lookupTransform("world", "qr_code", ros::Time(0), world_qr_stransform);
			//world_qr_transform = world_qr_stransform;
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(0.5).sleep();
			continue;
		}
		if(status.data == 3)world_goal_transform = world_qr_stransform*qr_goal_transform;
		br.sendTransform(tf::StampedTransform(world_goal_transform, ros::Time::now(), "world", "goal"));
	}
	ros::spin();
	return 0;
}

void qrPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	//ROS_INFO("data");
	stamp = msg->header.stamp;
	if(status.data == 3){
	camera_qrcode_transform.setOrigin(tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
	camera_qrcode_transform.setRotation(tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w));
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
		status.data = 0;
	}
	if(count>=3)
	{
		status.data = 3;
		count = 3;
	}
	ROS_INFO("count: %d", count);
}
