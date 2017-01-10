##! /usr/bin/env python
# Filename: parse_vel.py
# Author: Dave

import rospy
from geometry_msgs.msg import Twist
pub = None
vx = 0;
vy = 0;
w = 0;

def callback(data):
	global pub, vx, vy, w
	if(data.linear.x > 0):
		vx += 0.2;
	if(data.linear.x < 0):
		vx -= 0.2;

	if(data.linear.y > 0):
		vy += 0.2;
	if(data.linear.y < 0):
		vy -= 0.2;
	if(data.angular.z > 0):
		w += 5;
	if(data.angular.z < 0):
		w -= 5;
	msg = Twist();
	msg.linear.x = vx;
	msg.linear.y = vy;
	msg.linear.z = 0;
	msg.angular.x = 0;
	msg.angular.y =0;
	msg.angular.z = w;
	pub.publish(msg)

def main():
	global pub
	try:
		rospy.init_node('agv_ctrl_interface', anonymous = True)
		rospy.Subscriber('turtle1/cmd_vel', Twist, callback)
		pub = rospy.Publisher('cmd_vel', Twist, queue_size=1000)
		rospy.spin()
	except KeyboardInterrupt:
		raise

if __name__=='__main__':main()
