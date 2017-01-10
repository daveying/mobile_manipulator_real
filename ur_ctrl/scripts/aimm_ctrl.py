#! /usr/bin/env python
# Filename: aimm_ctrl.py
# Author: Dave

import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *

import socket
import threading
import binascii

from ur_ctrl.msg import *

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

q = [2,0.5,-1,0,0,0]
client =None
start_time = None
t = 4

def callback(data):
	g = FollowJointTrajectoryGoal()
	g.trajectory = JointTrajectory()
	g.trajectory.joint_names = JOINT_NAMES
	g.trajectory.points = [JointTrajectoryPoint(positions = data.Q, velocities = [0] * 6, time_from_start = rospy.Duration(data.time))]
	client.send_goal(g)
	try:
		client.wait_for_result()
	except KeyboardInterrupt:
		client.cancel_goal()
		raise

def main():
	global client
	try:
		rospy.init_node("aimm_move", anonymous = True, disable_signals = True)
		client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
		print "Waiting for server..."
		client.wait_for_server()
		print "Connected to server"
		rospy.Subscriber("aimm_chatter", aimm_ctrl_msg, callback)
		rospy.spin()
	except KeyboardInterrupt:
		rospy.signal_shutdown("KeyboardInterrupt")
		raise

if __name__ == '__main__': main()


