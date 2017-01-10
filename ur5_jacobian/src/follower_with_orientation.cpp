#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <tf/transform_listener.h>

#include <cmath>
using namespace std;

std::vector<double> joint_speeds;
std::vector<double> joint_values;
bool joint_data_come = false;

int avarage_length = 20;
Eigen::MatrixXd joint_speed_c(6,avarage_length);
int indexx = 0;


ros::Subscriber sub_joint_states;
ros::Publisher pub_joint_speed;


/*callback functions*/
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
/*other functions*/
void updateDiff(tf::TransformListener &tf_listener, Eigen::Vector3d &diff_vector, tf::Vector3 &axis, double &shortest_angle);
void movingAvarage(Eigen::MatrixXd &joint_ctrl_velocities, std::vector<double> &result);
void integralDiff(Eigen::Vector3d &diff_vector, double shortest_angle, Eigen::Vector3d &integral_vector, double &integral_angle);
double calcNorm(Eigen::MatrixXd vector);
double calcNorm(tf::Vector3 vector);

int main(int argc, char** argv)
{
	joint_speeds.resize(6);
	joint_values.resize(6);

	for(int i = 0; i < avarage_length; ++i)
		for(int j = 0; j < 6; ++j)
			joint_speed_c(j, i) = 0;


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
	Eigen::Vector3d reference_point_position(0.0,0.0,0);
	Eigen::MatrixXd jacobian;	//Jacobian of current configuration

	Eigen::MatrixXd joint_ctrl_velocities(6,1);		//joint velocities required
	Eigen::MatrixXd tcp_velocities_world(6,1);		//tcp velocities calculated by PID controller
	Eigen::Vector3d diff_vector(0.0, 0.0, 0.0);	 	//position Kp
	Eigen::Vector3d integral_vector(0.0, 0.0, 0.0); //position Ki

	tf::Vector3 axis(0, 0, 0); 	//orientation Kp
	double shortest_angle = 0; 	//orientation Kp
	double integral_angle = 0;  //orientation Ki
	tf::Vector3 t_w_wxyz;

	/*for command send*/
	trajectory_msgs::JointTrajectory trj;
	trajectory_msgs::JointTrajectoryPoint trjp;
	trjp.velocities.resize(6);
	trj.points.push_back(trjp);

	//TODO, Paramters of PID controller
	double Kp_pose = 2, Ki_pose = 0/*0.001/5*/, Kp_orien = 1.80, Ki_orien = 0.000;

	tf::TransformListener tf_listener;
	
	updateDiff(tf_listener, diff_vector, axis, shortest_angle);
	integralDiff(diff_vector, shortest_angle, integral_vector, integral_angle);

	while(ros::ok())
	{
		if(joint_data_come)
		{
			joint_data_come = false;
			//calculate jacobian
			kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
			kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, jacobian);
			//ROS_INFO_STREAM("Jacobian: \n" << jacobian);
			//end calculate jacobian

			//calculate TCP velocities
			tcp_velocities_world(0,0) = Kp_pose*diff_vector(0) + Ki_pose * integral_vector(0);
			tcp_velocities_world(1,0) = Kp_pose*diff_vector(1) + Ki_pose * integral_vector(1);
			tcp_velocities_world(2,0) = Kp_pose*diff_vector(2) + Ki_pose * integral_vector(2);
			double linear_v = calcNorm(tcp_velocities_world);
			double max_linear_speed = 0.5; //TODO, change the linear speed limit
			if(linear_v > max_linear_speed)
			{
				tcp_velocities_world(0,0) = tcp_velocities_world(0,0) * max_linear_speed / linear_v;
				tcp_velocities_world(1,0) = tcp_velocities_world(1,0) * max_linear_speed / linear_v;
				tcp_velocities_world(2,0) = tcp_velocities_world(2,0) * max_linear_speed / linear_v;
			}


			double angular_v = shortest_angle*Kp_orien + integral_angle * Ki_orien;
			double max_angular_speed = 0.1; //TODO, change the angular speed limit
			if(angular_v > max_angular_speed)
			{
				angular_v = max_angular_speed;
			}


			t_w_wxyz = axis * angular_v;

			tcp_velocities_world(3,0) = t_w_wxyz[0];
			tcp_velocities_world(4,0) = t_w_wxyz[1];
			tcp_velocities_world(5,0) = t_w_wxyz[2];
			//end calculate TCP velocities

			//calculate joint velocities
			joint_ctrl_velocities = jacobian.inverse() * tcp_velocities_world;
			//moving averages
			movingAvarage(joint_ctrl_velocities, trj.points[0].velocities);
			//end moving averages

			// joint speed limit
			double max_joint_speed = 0.5;
			ROS_INFO("%lf, %lf, %lf, %lf, %lf, %lf\n", trj.points[0].velocities[0],trj.points[0].velocities[2],trj.points[0].velocities[2],trj.points[0].velocities[3],trj.points[0].velocities[4],trj.points[0].velocities[5]);
			for(int i = 0; i < 6; ++i)
			{
				//if(trj.points[0].velocities[i] > max_joint_speed)
					//trj.points[0].velocities[i] = max_joint_speed;
			}
		}
		updateDiff(tf_listener, diff_vector, axis, shortest_angle);
		integralDiff(diff_vector, shortest_angle, integral_vector, integral_angle);

		trj.header.stamp = ros::Time::now();
		pub_joint_speed.publish(trj);
		ros::spinOnce();
		loop_rate.sleep();
	}
}

double calcNorm(Eigen::MatrixXd vector)
{
	return sqrt(vector(0,0)*vector(0,0) + vector(1,0)*vector(1,0) + vector(2,0)*vector(2,0));
}

double calcNorm(tf::Vector3 vector)
{
	return sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]);
}

void integralDiff(Eigen::Vector3d &diff_vector, double shortest_angle, Eigen::Vector3d &integral_vector, double &integral_angle)
{
	//TODO, change the integral time
	int count = 150; //100Hz, 50 is represents 0.5s
	static std::vector<Eigen::Vector3d> vector_c(count, Eigen::Vector3d(0, 0, 0));
	static int index = 0;
	vector_c[index] = diff_vector;
	index++;
	if(index == count)index = 0;
	for(int i = 0; i != vector_c.size(); ++i)
		integral_vector += vector_c[i];

	static std::vector<double> angle_c(count, 0);
	static int index2 = 0;
	angle_c[index2] = shortest_angle;
	index2++;
	if(index2 == count)index2 = 0;
	for(int i = 0; i != angle_c.size(); ++i)
		integral_angle += angle_c[i];
}

void movingAvarage(Eigen::MatrixXd &joint_ctrl_velocities, std::vector<double> &result)
{

	for(int ii = 0; ii<6; ii++)
	{
		joint_speed_c(ii, indexx) = joint_ctrl_velocities(ii, 0);
	}
	indexx = indexx + 1;
	//std::cout << "joint_speed_c: " << joint_speed_c << std::endl;
	//std::cout << "indexx: " << indexx << std::endl;
	if(indexx==avarage_length)indexx=0;
	double v1=0, v2=0, v3=0, v4 = 0, v5=0, v0=0;
	for(int iii=0; iii<avarage_length; iii++)
	{
		v0+=joint_speed_c(0,iii);
		v1+=joint_speed_c(1,iii);
		v2+=joint_speed_c(2,iii);
		v3+=joint_speed_c(3,iii);
		v4+=joint_speed_c(4,iii);
		v5+=joint_speed_c(5,iii);
	}
	v0/=avarage_length;
	v1/=avarage_length;
	v2/=avarage_length;
	v3/=avarage_length;
	v4/=avarage_length;
	v5/=avarage_length;
	result[0] = v0;
	result[1] = v1;
	result[2] = v2;
	result[3] = v3;
	result[4] = v4;
	result[5] = v5;
}

void updateDiff(tf::TransformListener &tf_listener, Eigen::Vector3d &diff_vector, tf::Vector3 &axis, double &shortest_angle)
{	

	tf::StampedTransform g_w_transform;//goal respect to world
	tf::StampedTransform t_w_transform;//tool respect to world
	tf::Transform g_t_transform;//goal respect to tool
	tf::Quaternion g_t_rotation;

	bool s = false;
	while(!s && ros::ok())
	{
		try
		{// ur_base_link --> base_link, goal --> ar_grasp_e
			tf_listener.lookupTransform("ur_base_link", "tool0", ros::Time(0), t_w_transform);
			tf_listener.lookupTransform("ur_base_link", "goal", ros::Time(0), g_w_transform);
			g_t_transform = t_w_transform.inverse() * g_w_transform;
			g_t_rotation = g_t_transform.getRotation();

			//third return value
			shortest_angle = g_t_rotation.getAngle();
			//ROS_INFO("angle: %lf", shortest_angle);
			if(shortest_angle < 0.001)shortest_angle = 0;

			tf::StampedTransform t_w_rotation = t_w_transform;
			t_w_rotation.setOrigin(tf::Vector3(0,0,0));

			//second return value
			axis = t_w_rotation * g_t_rotation.getAxis();

			//first return value
			diff_vector(0) = g_w_transform.getOrigin().x() - t_w_transform.getOrigin().x();
			diff_vector(1) = g_w_transform.getOrigin().y() - t_w_transform.getOrigin().y();
			diff_vector(2) = g_w_transform.getOrigin().z() - t_w_transform.getOrigin().z();
			//ROS_INFO("diff vector:%lf, %lf, %lf\n" , diff_vector(0), diff_vector(1), diff_vector(2));
			s = true;
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1).sleep();
			continue;
		}
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
		joint_data_come = true;
	}
}

