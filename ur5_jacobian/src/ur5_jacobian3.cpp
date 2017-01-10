#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <trajectory_msgs/JointTrajectory.h>

std::vector<double> joint_speeds;
std::vector<double> joint_values;
std::vector<double> mouse_speeds;//mouse_speeds[0] is x direction velocity
bool joint_data_come = false;
bool mouse_data_come = false;
std_msgs::Header header;

ros::Subscriber sub_joint_states;
ros::Subscriber sub_platform_speed;
ros::Publisher pub_joint_speed;
ros::Publisher pub_tcp_speed;

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	if(msg->velocity.size() == 6)
	{
		//ROS_INFO("velocities: %lf, %lf, %lf, %lf, %lf, %lf", msg->velocity[0], msg->velocity[1], msg->velocity[2], msg->velocity[3], msg->velocity[4], msg->velocity[5]);
		joint_speeds[0] = msg->velocity[0];
		joint_speeds[1] = msg->velocity[1];
		joint_speeds[2] = msg->velocity[2];
		joint_speeds[3] = msg->velocity[3];
		joint_speeds[4] = msg->velocity[4];
		joint_speeds[5] = msg->velocity[5];
		joint_values[0] = msg->position[0];
		joint_values[1] = msg->position[1];
		joint_values[2] = msg->position[2];
		joint_values[3] = msg->position[3];
		joint_values[4] = msg->position[4];
		joint_values[5] = msg->position[5];
		header = msg->header;
		joint_data_come = true;
		
	}
}

void mouseSpeedsCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	mouse_speeds[0] = msg->twist.angular.x;
	mouse_speeds[1] = msg->twist.angular.y;
	mouse_speeds[2] = 0;
	mouse_data_come = true;
}

int main(int argc, char** argv)
{
	joint_speeds.push_back(0);
	joint_speeds.push_back(0);
	joint_speeds.push_back(0);
	joint_speeds.push_back(0);
	joint_speeds.push_back(0);
	joint_speeds.push_back(0);
	joint_values.push_back(0);
	joint_values.push_back(0);
	joint_values.push_back(0);
	joint_values.push_back(0);
	joint_values.push_back(0);
	joint_values.push_back(0);
	mouse_speeds.push_back(0);
	mouse_speeds.push_back(0);
	mouse_speeds.push_back(0);
	ros::init(argc, argv, "ur_jacobian_ctrl");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);
	ros::AsyncSpinner spinner(1);
	spinner.start();
	sub_joint_states = n.subscribe("/joint_states", 1000, jointStateCallback);
	sub_platform_speed = n.subscribe("/mouse_speeds", 1000, mouseSpeedsCallback);
	pub_tcp_speed = n.advertise<geometry_msgs::TwistStamped>("/tcp_velocities", 1000);
	pub_joint_speed = n.advertise<trajectory_msgs::JointTrajectory>("/ur_driver/joint_speed", 1000);
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	ROS_INFO("Model frame(base frame): %s", kinematic_model->getModelFrame().c_str());

	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
	const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
	Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
	Eigen::MatrixXd jacobian;
	Eigen::MatrixXd joint_velocities(6, 1);
	Eigen::MatrixXd joint_ctrl_velocities(6, 1);
	Eigen::MatrixXd tcp_velocities(6,1);
	Eigen::MatrixXd platform_velocities(6,1);
	geometry_msgs::TwistStamped tcp_msg;
	trajectory_msgs::JointTrajectory trj;
	trajectory_msgs::JointTrajectoryPoint trjp;
	trjp.velocities.push_back(0.0);
	trjp.velocities.push_back(0.0);
	trjp.velocities.push_back(0.0);
	trjp.velocities.push_back(0.0);
	trjp.velocities.push_back(0.0);
	trjp.velocities.push_back(0.0);
	trj.points.push_back(trjp);
	while(ros::ok())
	{
		if(mouse_data_come||joint_data_come)
		{
			mouse_data_come = false; joint_data_come = false;
			kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
			kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, jacobian);
			ROS_INFO_STREAM("Jacobian: \n" << jacobian);
			joint_velocities(0,0) = joint_speeds[0];
			joint_velocities(1,0) = joint_speeds[1];
			joint_velocities(2,0) = joint_speeds[2];
			joint_velocities(3,0) = joint_speeds[3];
			joint_velocities(4,0) = joint_speeds[4];
			joint_velocities(5,0) = joint_speeds[5];
			tcp_velocities = jacobian * joint_velocities;
			ROS_INFO_STREAM("TCP velocities: \n" << tcp_velocities);
			tcp_msg.header = header;
			tcp_msg.twist.linear.x = tcp_velocities(0,0);
			tcp_msg.twist.linear.y = tcp_velocities(1,0);
			tcp_msg.twist.linear.z = tcp_velocities(2,0);
			tcp_msg.twist.angular.x = tcp_velocities(3,0);
			tcp_msg.twist.angular.y = tcp_velocities(4,0);
			tcp_msg.twist.angular.z = tcp_velocities(5,0);
			pub_tcp_speed.publish(tcp_msg);
			double k=0.05;
			platform_velocities(0,0) = k*mouse_speeds[0];
			platform_velocities(1,0) = k*mouse_speeds[1];
			platform_velocities(2,0) = 0;
			platform_velocities(3,0) = 0;
			platform_velocities(4,0) = 0;
			platform_velocities(5,0) = 0;
			
			joint_ctrl_velocities = jacobian.inverse() * platform_velocities;
			trj.points[0].velocities[0] = joint_ctrl_velocities(0,0);
			trj.points[0].velocities[1] = joint_ctrl_velocities(1,0);
			trj.points[0].velocities[2] = joint_ctrl_velocities(2,0);
			trj.points[0].velocities[3] = joint_ctrl_velocities(3,0);
			trj.points[0].velocities[4] = joint_ctrl_velocities(4,0);
			trj.points[0].velocities[5] = joint_ctrl_velocities(5,0);
		}	
		trj.header.stamp = ros::Time::now();
		pub_joint_speed.publish(trj);
		ros::spinOnce();
		loop_rate.sleep();
	}




















	
}
