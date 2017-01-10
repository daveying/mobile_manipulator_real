#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <tf/transform_listener.h>

std::vector<double> joint_speeds;
std::vector<double> joint_values;
bool joint_data_come = false;
std_msgs::Header header;

ros::Subscriber sub_joint_states;
ros::Publisher pub_joint_speed;


geometry_msgs::Pose goal_in_world;

/*callback functions*/
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

int main(int argc, char** argv)
{
	joint_speeds.resize(6);
	joint_values.resize(6);


	ros::init(argc, argv, "ur5_jacobian_follower");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);
	ros::AsyncSpinner spinner(1);
	spinner.start();
	sub_joint_states = n.subscribe("/joint_states", 1000, jointStateCallback);
	pub_joint_speed = n.advertise<trajectory_msgs::JointTrajectory>("/ur_driver/joint_speed", 1000);
	/*for jacobian calculation*/
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	ROS_INFO("Model frame(base frame): %s", kinematic_model->getModelFrame().c_str());
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
	/*for velocities calculation*/
	Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
	Eigen::MatrixXd jacobian;
	Eigen::MatrixXd joint_velocities(6,1);
	Eigen::MatrixXd joint_ctrl_velocities(6,1);
	Eigen::MatrixXd tcp_velocities_world(6,1);
	Eigen::Vector3d diff_vector(0.0, 0.0, 0.0);
	/*for command send*/
	trajectory_msgs::JointTrajectory trj;
	trajectory_msgs::JointTrajectoryPoint trjp;
	trjp.velocities.resize(6);
	trj.points.push_back(trjp);
	/*for goal acquire*/
	tf::TransformListener tf_listener;
	tf::StampedTransform g_w_transform;//goal respect to world
	tf::StampedTransform t_w_transform;//tool respect to world
	tf::Transform g_t_transform;//goal respect to tool
	tf::Quaternion g_t_rotation;
	tf::Vector3 t_w_wxyz;
	tf::Vector3 t_t_wxyz;
	double kp = 3, ko = 1.8;//TODO
	bool s = false;
	while(!s && ros::ok())
	{
		try
		{
			tf_listener.lookupTransform("world", "tool0", ros::Time(0), t_w_transform);
			tf_listener.lookupTransform("world", "goal", ros::Time(0), g_w_transform);
			g_t_transform = t_w_transform.inverse() * g_w_transform;
			g_t_rotation = g_t_transform.getRotation();
			double shortest_angle = g_t_rotation.getAngle();
			ROS_INFO("angle: %lf", shortest_angle);
			if(shortest_angle < 0.001)shortest_angle = 0;
			t_t_wxyz = g_t_rotation.getAxis() * shortest_angle * ko;//TODO
			tf::StampedTransform t_w_rotation = t_w_transform;
			t_w_rotation.setOrigin(tf::Vector3(0,0,0));
			t_w_wxyz = t_w_rotation * t_t_wxyz;
			diff_vector(0) = g_w_transform.getOrigin().x() - t_w_transform.getOrigin().x();
			diff_vector(1) = g_w_transform.getOrigin().y() - t_w_transform.getOrigin().y();
			diff_vector(2) = g_w_transform.getOrigin().z() - t_w_transform.getOrigin().z();
			ROS_INFO("diff vector:%lf, %lf, %lf\n" , diff_vector(0), diff_vector(1), diff_vector(2));
			s = true;
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1).sleep();
			continue;
		}
	}
	ROS_INFO("sds");
	Eigen::MatrixXd joint_speed_c(6,10);
	for(int i = 0;i<6;i++)
		for(int j = 0;j<10;j++)
			joint_speed_c(i,j) = 0;
	int index = 0;
	while(ros::ok())
	{
		if(joint_data_come)
		{
			joint_data_come = false;
			kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
			kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, jacobian);
			ROS_INFO_STREAM("Jacobian: \n" << jacobian);
			tcp_velocities_world(0,0) = kp*diff_vector(0);// + kd*tcp_velocities_world(0,0);
			tcp_velocities_world(1,0) = kp*diff_vector(1);// + kd*tcp_velocities_world(1,0);
			tcp_velocities_world(2,0) = kp*diff_vector(2);// + kd*tcp_velocities_world(2,0);
			tcp_velocities_world(3,0) = t_w_wxyz[0];
			tcp_velocities_world(4,0) = t_w_wxyz[1];
			tcp_velocities_world(5,0) = t_w_wxyz[2];
			joint_ctrl_velocities = jacobian.inverse() * tcp_velocities_world;
			//moving averages
			for(int ii = 0; ii<6; ii++)
			{
				joint_speed_c(ii, index) = joint_ctrl_velocities(ii, 0);	
			}
			index++;
			if(index==10)index=0;
			double v1=0, v2=0, v3=0, v4 = 0, v5=0, v0=0;
			for(int iii=0; iii<10; iii++)
			{
				v0+=joint_speed_c(0,iii);
				v1+=joint_speed_c(1,iii);
				v2+=joint_speed_c(2,iii);
				v3+=joint_speed_c(3,iii);
				v4+=joint_speed_c(4,iii);
				v5+=joint_speed_c(5,iii);
			}
			v0/=10;
			v1/=10;
			v2/=10;
			v3/=10;
			v4/=10;
			v5/=10;
			trj.points[0].velocities[0] = v0;
			trj.points[0].velocities[1] = v1;
			trj.points[0].velocities[2] = v2;
			trj.points[0].velocities[3] = v3;
			trj.points[0].velocities[4] = v4;
			trj.points[0].velocities[5] = v5;
			//end moving averages
		}
		s = false;
		while(!s && ros::ok())
		{
			try
			{
				tf_listener.lookupTransform("world", "tool0", ros::Time(0), t_w_transform);
				tf_listener.lookupTransform("world", "goal", ros::Time(0), g_w_transform);
				g_t_transform = t_w_transform.inverse() * g_w_transform;
				g_t_rotation = g_t_transform.getRotation();
				double shortest_angle = g_t_rotation.getAngle();
				ROS_INFO("angle: %lf", shortest_angle);
				if(shortest_angle < 0.001)shortest_angle = 0;
				t_t_wxyz = g_t_rotation.getAxis() * shortest_angle * ko;//TODO
				tf::StampedTransform t_w_rotation = t_w_transform;
				t_w_rotation.setOrigin(tf::Vector3(0,0,0));
				t_w_wxyz = t_w_rotation * t_t_wxyz;
				diff_vector(0) = g_w_transform.getOrigin().x() - t_w_transform.getOrigin().x();
				diff_vector(1) = g_w_transform.getOrigin().y() - t_w_transform.getOrigin().y();
				diff_vector(2) = g_w_transform.getOrigin().z() - t_w_transform.getOrigin().z();
				//ROS_INFO_STREAM("diff vector:\n" << diff_vector);
				ROS_INFO("diff vector:%lf, %lf, %lf\n" , diff_vector(0), diff_vector(1), diff_vector(2));
				s = true;
			}
			catch (tf::TransformException &ex)
			{
				ROS_ERROR("%s", ex.what());
			}
		}
		trj.header.stamp = ros::Time::now();
		pub_joint_speed.publish(trj);
		ros::spinOnce();
		loop_rate.sleep();
	}
}


/*callback functions*/
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

