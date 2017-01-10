#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>

tf::Transform optical_qrcode_transform;
ros::Time stamp;
std_msgs::Int8 status;
tf::TransformListener* plistener;

int count = 0;


tf::StampedTransform world_optical_stransform;

bool first1 = true;
bool first = true;

void qrPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void statusCallback(const std_msgs::Int8::ConstPtr& msg);
void distCallback(const std_msgs::Float64::ConstPtr& msg);

double dist = 0.4;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "qrcode_stable_goal_broadcaster");
	ros::NodeHandle n;
	tf::TransformBroadcaster br;
	tf::TransformListener listener;
	plistener = &listener;
	status.data = 0;
	
	ros::Subscriber sub_qr_pose = n.subscribe("/visp_auto_tracker/object_position", 1000, qrPoseCallback);
	ros::Subscriber sub_status = n.subscribe("/visp_auto_tracker/status", 1000, statusCallback);
	ros::Subscriber sub_dist = n.subscribe("/goal_distance", 1000, distCallback);


	tf::Transform qr_goal_transform;
	ros::Rate rate(10);
	
	qr_goal_transform.setOrigin(tf::Vector3(0.02, 0.08, dist));
	qr_goal_transform.setRotation(tf::Quaternion(1 , 0, 0, 0));
	optical_qrcode_transform.setOrigin(tf::Vector3(0,0,0));
	optical_qrcode_transform.setRotation(tf::Quaternion(0, 0, 0, 1));


	//tf::StampedTransform world_qr_stransform;
	tf::Transform world_goal_transform;

	
	while(ros::ok())
	{
		rate.sleep();
		ros::spinOnce();

		if(first == false)
		{
			qr_goal_transform.setOrigin(tf::Vector3(0.02, 0.08, dist));
			qr_goal_transform.setRotation(tf::Quaternion(1 , 0, 0, 0));
			world_goal_transform = world_optical_stransform * optical_qrcode_transform * qr_goal_transform;
			br.sendTransform(tf::StampedTransform(world_goal_transform, ros::Time::now(), "world", "goal"));
		ROS_INFO("sddd");
		}

	}
	ros::spin();
	return 0;
}

void distCallback(const std_msgs::Float64::ConstPtr &msg)
{
	dist = msg->data;
	if(dist < 0.3)
		dist = 0.3;
}

void qrPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

	stamp = msg->header.stamp;
	if(status.data == 3 && first == true)
	{
		first = false;
		optical_qrcode_transform.setOrigin(tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
		optical_qrcode_transform.setRotation(tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w));
		bool s = false;
		while(ros::ok() && !s)
		{		
			try
			{
				plistener->lookupTransform("world", "camera_rgb_optical_frame", ros::Time(0), world_optical_stransform);
				s = true;
				ROS_INFO("sss");
			}
			catch (tf::TransformException &ex)
			{
				ROS_ERROR("dd%s", ex.what());
				ros::Duration(0.5).sleep();
				//continue;
			}
		
		}
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
	if(count>=6)
	{
		status.data = 3;
		count = 6;
	}
	ROS_INFO("count: %d", count);
}
