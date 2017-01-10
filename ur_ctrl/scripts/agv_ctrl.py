##! /usr/bin/env python
# Filename: agv_ctrl.py
# Author: Dave

import agv_with_stm32
import time

import rospy
from geometry_msgs.msg import Twist
agv = None

def callback(data):
	global agv
	agv.set_velocities(data.linear.x,data.linear.y,data.angular.z)
	agv.send_velocities()
	time.sleep(0.2)
	agv.send_velocities()

def main():
	global agv
	try:
		agv = agv_with_stm32.AGV()
		rospy.init_node('agv_ctrl_interface',anonymous = True)
		rospy.Subscriber('agv/cmd_vel', Twist, callback)
		rospy.spin()
	except KeyboardInterrupt:
		raise

if __name__=='__main__':main()
