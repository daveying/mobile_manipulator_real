#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <car_serial/SerialPort.h>

#define FREQUENCY 10
#define COMMAND_SIZE 6 
 
#define MAXZERONUM 10

int flag=0;

class DataUpdater
{
public:
	DataUpdater(ros::NodeHandle &nh);
	~DataUpdater();
	geometry_msgs::Twist getCarVel();

private:
	// callback functions
	void carVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

	// subscribers
	ros::Subscriber carVelSub;

	// data
	geometry_msgs::Twist::ConstPtr carVel; //boost::shared_ptr<const geometry_msgs::Twist_<std::allocator<void> > >	
	// the latest time that recieve a message
	ros::Time carVelTime;
	int ACCEPTTIME;
};

DataUpdater::DataUpdater(ros::NodeHandle &nh)
{
	carVelSub = nh.subscribe("cmd_vel",1,&DataUpdater::carVelCallback,this);
	ACCEPTTIME = 200000000*100; // 0.5s, unit: ns(10^-9s) TODO: this may need change
}

DataUpdater::~DataUpdater()
{
}

void DataUpdater::carVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	flag=1;
	carVel = msg;
       // carVel->inear.x *= -1;
	//ROS_INFO("car_vel: %.4f, %.4f, %.4f", msg->linear.x, msg->linear.y, msg->angular.z);
	carVelTime = ros::Time::now();
}

geometry_msgs::Twist DataUpdater::getCarVel()
{
	ros::Time nowTime = ros::Time::now();
	ros::Duration timediff = nowTime-carVelTime;
	if(carVel)// && timediff.nsec<ACCEPTTIME)// && timediff.sec==0 ) // this message is new enough
	{
		return *carVel;
	}

	geometry_msgs::Twist dummyZero;
	//ROS_INFO("RETURN DUMMY");
	return dummyZero;
}

void genCmd(char* cmdData, double vx, double vy, double w)
{

	ROS_INFO("vx: %lf, vy: %lf, w: %lf", vx, vy, w);
	if (vx>5)vx = 5;
	if (vx<-5)vx = -5;
	if (vy>5)vy = 5;
	if (vy<-5)vy = -5;
	if (w>100)w = 100;
	if (w<-100)w = -100;

	cmdData[0] = (int8_t)0xfa;
	cmdData[1] = (int8_t)0xfb;
	cmdData[2] = (int8_t)0x01;
	int16_t ivx = int16_t(vx*1000);
	int16_t ivy = int16_t(vy*1000);
	int8_t iw = int8_t(w*180/3.14);//unit of w is rad/s, while unit of iw is degree/s
	cmdData[4] = int8_t(ivx & 0x00ff);
	cmdData[3] = int8_t((ivx >> 8) & 0x00ff);
	cmdData[6] = int8_t(ivy & 0x00ff);
	cmdData[5] = int8_t((ivy >> 8) & 0x00ff);
	cmdData[7] = iw;
	cmdData[8] = int8_t(0xee);
	cmdData[9] = int8_t(0xff);
	cmdData[10] = int8_t(0x3f);
	cmdData[11] = int8_t(0x3b);

}

void velFilter(short &vx, short &va, int &count, short &lastvx, short &lastva)
{
	if(vx==0 && va==0)
	{
		count++;
		if(count>MAXZERONUM)
		{
			count = MAXZERONUM;
		}
		else
		{
			vx = lastvx;
			va = lastva;
			ROS_INFO("*****FILT OUT********");
		}
	}
	else
	{
		count = 0;
		lastvx = vx;
		lastva = va;
	}
}

int main(int argc, char **argv)
{
    char* CommPort = "/dev/ttyUSB0";//"/dev/ttyS0"; //default port

    ros::init(argc, argv, "car_serial");
    ros::NodeHandle nh;
    DataUpdater dataUpdater(nh);

    serial::SerialPort device;
    char cmd[12];

    // Change the next line according to your port name and baud rate
    try
    { 
    	device.open(CommPort, 115200); // TODO:
    }  
    catch(serial::Exception& e)
    {
        ROS_FATAL("Failed to open the serial port!!!");
        ROS_BREAK();
    }
    ROS_INFO("The serial port is open...");

    ros::Rate rate(FREQUENCY); // TODO: 

    geometry_msgs::Twist carVelocity;
    int zeroCount = 0;
    //short lastvx=0;
    //short lastva=0;
    //short vx,va;
    double velx,velw,vely;
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
		// Read the velocity
		carVelocity = dataUpdater.getCarVel();

		velx = carVelocity.linear.x;
		vely = carVelocity.linear.y;
		velw = carVelocity.angular.z;

		genCmd(cmd,velx,vely,velw);

		try{
			device.write(cmd,12);
			ROS_INFO("Send cmd %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x.",cmd[0],cmd[1],cmd[2],cmd[3],cmd[4],cmd[5],cmd[6],cmd[7],cmd[8],cmd[9],cmd[10],cmd[11]);
			//char rev[12];
			//int lenn = device.readBytes(rev, 12);
			//ROS_INFO("Receive data %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x.",rev[0],rev[1],rev[2],rev[3],rev[4],rev[5],rev[6],rev[7],rev[8],rev[9],rev[10],rev[11]);
		}catch(serial::Exception e)
		{
		  ROS_INFO("Write error..");
		}


    }   
}

