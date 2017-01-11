#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped
import pymouse


def talker():
	pub = rospy.Publisher("mouse_speeds", TwistStamped, queue_size = 10)
	rospy.init_node('mouse_talker')
	rate = rospy.Rate(100)
	m = pymouse.PyMouse()
	pos = []
	pos_ = []
	_pos = []
	i = 0
	while not rospy.is_shutdown():
		pos_ = m.position()
		if i >= 2:
			_vx = (pos[0] - _pos[0])/0.01
			_vy = (pos[1] - _pos[1])/0.01
			vx_ = (pos_[0] - pos[0])/0.01
			vy_ = (pos_[1] - pos[1])/0.01
			vx = (vx_+_vx)/2000
			vy = (vy_+_vy)/2000
			info_str = "x: "+pos[0].__str__()+" y: "+pos[1].__str__()+" vx: "+vx.__str__()+" vy: "+vy.__str__()
			rospy.loginfo(info_str)
			mouse_msg = TwistStamped()
			mouse_msg.twist.linear.x = pos[0]/1000.0
			mouse_msg.twist.linear.y = pos[1]/1000.0
			mouse_msg.twist.angular.x = vx
			mouse_msg.twist.angular.y = vy
			pub.publish(mouse_msg)
		_pos = pos
		pos = pos_
		i = i+1
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
