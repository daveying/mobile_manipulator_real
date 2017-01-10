#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

//#include <vector>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");

  ros::NodeHandle n;
  ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
  ros::Rate loop_rate(10);
  int count = 0;
  sensor_msgs::JointState msg;
  msg.name.resize(6);
  msg.position.resize(6);
  msg.name[0] = "shoulder_pan_joint";
  msg.name[1] = "shoulder_lift_joint";
  msg.name[2] = "elbow_joint";
  msg.name[3] = "wrist_1_joint";
  msg.name[4] = "wrist_2_joint";
  msg.name[5] = "wrist_3_joint";
  msg.position[0] = 2.0;
  msg.position[1] = 1.0;
  msg.position[2] = 2.0;
  msg.position[3] = 0.5;
  msg.position[4] = 1.9;
  msg.position[5] = 1.0;
  //sensor_msgs::JointState::_name_type _name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  //std::vector<string> _name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  while(ros::ok())
  {
    //sensor_msgs::JointState msg;
    //msg.name = _name;
    //msg.position = {"2.0", "2.0", "2.0", "2.0", "2.0", "2.0"};
    msg.header.stamp = ros::Time::now();
    joint_state_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
}
