#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>

std::vector<double> joint_speeds;
std::vector<double> joint_values;
bool data_come = false;
std_msgs::Header header;

void chatterCallback(const sensor_msgs::JointState::ConstPtr& msg)
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
		data_come = true;
		
	}
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
	ros::init(argc, argv, "ur_jacobian_test");
	ros::NodeHandle n;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::Subscriber sub = n.subscribe("/joint_states", 1000, chatterCallback);
	ros::Publisher pub= n.advertise<geometry_msgs::TwistStamped>("/tcp_velocity", 1000);
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
	const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
	Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
	Eigen::MatrixXd jacobian;
	Eigen::MatrixXd joint_velocities(6,1);
	Eigen::MatrixXd tcp_velocities(6,1);
	geometry_msgs::TwistStamped tcp_msg;
	while(ros::ok())
	{
		if(data_come)
		{
			data_come = false;
			//ROS_INFO("v: %lf, %lf, %lf, %lf, %lf, %lf", joint_speeds[0], joint_speeds[1], joint_speeds[2], joint_speeds[3], joint_speeds[4], joint_speeds[5]);
			kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
			
			kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, jacobian);
			ROS_INFO_STREAM("Jacobian: \n" << jacobian);
			ROS_INFO_STREAM("Inverse of Jacobian: \n" << jacobian.inverse());
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
			pub.publish(tcp_msg);
		}
	}
	ros::spin();
}
