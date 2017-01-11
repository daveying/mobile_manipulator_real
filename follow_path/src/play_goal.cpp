#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TwistStamped.h>
#include <vector>
using namespace std;
#define PI 3.1415926

vector< tf::Transform > path;
vector<double> point;


void initPath()
{
    point.resize(7);
    tf::Transform trsfm;

    point[0] = 0.638962; point[1] = 0.761629; point[2] = 0.107933; 
    point[3] = 0.000000; point[4] = 0.596201; point[5] = -0.315478; 
    point[6] = 0.240793;
    trsfm.setRotation(tf::Quaternion(point[0], point[1], point[2], point[3]));
    trsfm.setOrigin(tf::Vector3(point[4], point[5], point[6]));
    path.push_back(trsfm);path.push_back(trsfm);path.push_back(trsfm);


    point[0] = 0.638975; point[1] = 0.761611; point[2] = 0.107978; 
    point[3] = 0.000000; point[4] = 0.649299; point[5] = -0.315567; 
    point[6] = 0.157849;
    trsfm.setRotation(tf::Quaternion(point[0], point[1], point[2], point[3]));
    trsfm.setOrigin(tf::Vector3(point[4], point[5], point[6]));
    path.push_back(trsfm);


    point[0] = 0.638950; point[1] = 0.761604; point[2] = 0.108174; 
    point[3] = 0.000000; point[4] = 0.653941; point[5] = -0.181970; 
    point[6] = 0.157254;
    trsfm.setRotation(tf::Quaternion(point[0], point[1], point[2], point[3]));
    trsfm.setOrigin(tf::Vector3(point[4], point[5], point[6]));
    path.push_back(trsfm);


    point[0] = 0.638933; point[1] = 0.761561; point[2] = 0.108580; 
    point[3] = 0.000000; point[4] = 0.662092; point[5] = 0.018420; 
    point[6] = 0.157786;
    trsfm.setRotation(tf::Quaternion(point[0], point[1], point[2], point[3]));
    trsfm.setOrigin(tf::Vector3(point[4], point[5], point[6]));
    path.push_back(trsfm);


    point[0] = 0.639066; point[1] = 0.761526; point[2] = 0.108040; 
    point[3] = 0.000000; point[4] = 0.668070; point[5] = 0.192754; 
    point[6] = 0.157444;
    trsfm.setRotation(tf::Quaternion(point[0], point[1], point[2], point[3]));
    trsfm.setOrigin(tf::Vector3(point[4], point[5], point[6]));
    path.push_back(trsfm);


    point[0] = 0.639041; point[1] = 0.761524; point[2] = 0.108201; 
    point[3] = 0.000000; point[4] = 0.616210; point[5] = 0.192698; 
    point[6] = 0.251030;
    trsfm.setRotation(tf::Quaternion(point[0], point[1], point[2], point[3]));
    trsfm.setOrigin(tf::Vector3(point[4], point[5], point[6]));
    path.push_back(trsfm);

}

int i = 0;

void chatterCallback(const std_msgs::Empty msg)
{
    i = 0;
}

int main(int argc, char** argv)
{
    initPath();
	ros::init(argc, argv, "goal_tf_broadcaster");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("reset", 1000, chatterCallback);
	tf::TransformBroadcaster br;
	tf::TransformListener listener;
	tf::Transform transform;
	tf::Transform init_transform;
	tf::StampedTransform stransform;
	
	ros::Rate rate(10);
	while(ros::ok())
	{
		br.sendTransform(tf::StampedTransform(path[i/10], ros::Time::now(), "ur_base_link", "goal"));
		ros::spinOnce();
		rate.sleep();
		i++;
		if(i/10 == path.size())i--;
	}
	ros::spin();
	return 0;
}
