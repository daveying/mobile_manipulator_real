#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Empty.h>
#include <fstream>
#include <iostream>
using namespace std;

ofstream ofile;
tf::TransformListener* plistener;
int couunt = 0;

void chatterCallback(const std_msgs::Empty msg)
{	
    tf::Transform transform;
	tf::StampedTransform stransform;
	ofile.open("way_points.txt", ios::app);
	bool s = false;

	while(!s && ros::ok())
	{
		try
		{
			plistener->lookupTransform("/ur_base_link", "/tool0", ros::Time(0), stransform);
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
	tf::Matrix3x3 rotation = transform.getBasis();
	tf::Quaternion q = transform.getRotation();
	tf::Vector3 axis = q.getAxis();
	double w = q.getW();
	tf::Vector3 position = transform.getOrigin();
	//ROS_INFO("transform:\n - rotation:\n   %lf, %lf, %lf\n   %lf, %lf, %lf\n   %lf, %lf, %lf\n - position:\n   %lf, %lf, %lf\n", rotation[0][0], rotation[0][1], rotation[0][2], rotation[1][0], rotation[1][1], rotation[1][2], rotation[2][0], rotation[2][1], rotation[2][2], position[0], position[1], position[2]);
	if (couunt == 0)
	    cout << "\npoint.resize(7);\ntf::Transform trsfm;\n";
	couunt++;
	ROS_INFO("\npoint[0] = %lf; point[1] = %lf; point[2] = %lf; \npoint[3] = %lf; point[4] = %lf; point[5] = %lf; \npoint[6] = %lf;", axis[0], axis[1], axis[2], axis[3], position[0], position[1], position[2]);
	cout << "\ntrsfm.setRotation(tf::Quaternion(point[0], point[1], point[2], point[3]));\ntrsfm.setOrigin(tf::Vector3(point[4], point[5], point[6]));\npath.push_back(trsfm);";
	ofile << rotation[0][0] << " " << rotation[0][1] << " " << rotation[0][2] << " " << rotation[1][0] << " " << rotation[1][1] << " " << rotation[1][2] << " " << rotation[2][0] << " " << rotation[2][1] << " " << rotation[2][2] << " " << position[0] << " " << position[1] << " " << position[2] << " " << endl;
	ofile.close();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "way_point_record");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("record", 1000, chatterCallback);
	
	plistener = new tf::TransformListener();

	ros::spin();
	return 0;

}
