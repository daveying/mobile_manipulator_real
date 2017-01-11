#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped
import pymouse

def talker():
	pub = rospy.Publisher("mouse_speeds", TwistStamped, queue_size = 10)
	rospy.init_node("mouse_talker")
	rate = rospy.Rate(100)
	m = pymouse.PyMouse()
	pos = [0.0, 0.0]
	pos_ = []
	c_vx = []
	c_vy = []

	index = 0
	num = 10;
	
	for ii in range(num):
		c_vx.append(0.0)
		c_vy.append(0.0)
	while not rospy.is_shutdown():
		pos_ = m.position()
		vx = (pos_[0] - pos[0])/0.01
		vy = (pos_[1] - pos[1])/0.01
		pos = pos_
		c_vx[index] = vx
		c_vy[index] = vy
		mouse_msg = TwistStamped()
		mouse_msg.twist.linear.x = pos[0]/2000.0 - 0.9595/2
		mouse_msg.twist.linear.y = pos[1]/2000.0 - 0.5395/2
		sum1 = 0
		sum2 = 0
		for iii in range(num):
			sum1 = sum1 + c_vx[iii]
			sum2 = sum2 + c_vy[iii]
		mouse_msg.twist.angular.x = sum1/(1000*num)
		mouse_msg.twist.angular.y = sum2/(1000*num)
		info_str = "x: "+mouse_msg.twist.linear.x.__str__()+" y: "+mouse_msg.twist.linear.y.__str__()+" vx: "+mouse_msg.twist.angular.x.__str__()+" vy: "+mouse_msg.twist.angular.y.__str__()
		rospy.loginfo(info_str)
		pub.publish(mouse_msg)
		index = index + 1
		if index == num:
			index = 0
		rate.sleep()


if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
		
