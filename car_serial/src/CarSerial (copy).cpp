#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <car_serial/SerialPort.h>

#define FREQUENCY 10
#define COMMAND_SIZE 6 
 
#define MAXZERONUM 10



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
		return *carVel;
	geometry_msgs::Twist dummyZero;
	//ROS_INFO("RETURN DUMMY");
	return dummyZero;
}

void genCmd(char* cmdData, short vx, short va)
{
	void* tempP1 = &vx;
	void* tempP2 = &va;
	char* velxChar = (char*)tempP1;
	char* velaChar = (char*)tempP2;

	cmdData[0] = (char)0xC3;
	cmdData[1] = velxChar[1]; //swap the high byte and the low byte
	cmdData[2] = velxChar[0];
	cmdData[3] = velaChar[1];
	cmdData[4] = velaChar[0];
	cmdData[5] = (char)0x0A; //

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
    char cmd[COMMAND_SIZE];

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
    short lastvx=0;
    short lastva=0;
    short vx,va;
    double velx,vela;
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        
	// Read the velocity
        carVelocity = dataUpdater.getCarVel();
        velx = -1*carVelocity.linear.x;
        vela = carVelocity.angular.z;
	vx = (int)(velx*1000.0);//for specific car only
	va = (int)(vela*1000.0);//suppose to be 1000

        //velFilter(vx,va,zeroCount,lastvx,lastva); // filt out single zero

        genCmd(cmd,vx,va);
	
        try{
            device.write(cmd,COMMAND_SIZE);
            ROS_INFO("Send cmd %x, %x, %x, %x, %x, %x.",cmd[0],cmd[1],cmd[2],cmd[3],cmd[4],cmd[5]);
        }catch(serial::Exception e)
        {
          ROS_INFO("Write error..");
        }
    }   
}

