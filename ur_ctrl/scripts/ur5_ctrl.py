#! /usr/bin/env python
# Filename: ur5_ctrl.py
# Author: Dave

#import ROS related module
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
#import AGV related module
import socket
import threading
import binascii

from ur_ctrl.msg import *

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
q = [1.5,0,-1.57,0,0,0]		#goal position of six joint. used by move(), modified by main()
client = None				#used by move(), created in main()
t = 4						#time to move to goal position.

def ur_move():
	g = FollowJointTrajectoryGoal()
	g.trajectory = JointTrajectory()
	g.trajectory.joint_names = JOINT_NAMES
	g.trajectory.points = [JointTrajectoryPoint(positions = q, velocities = [0] * 6, time_from_start = rospy.Duration(t))]
	client.send_goal(g)
	try:
		client.wait_for_result()
	except KeyboardInterrupt:
		client.cancel_goal()
		raise

def callback(data):
	q = data.Q

def main():
	global client
	try:
		rospy.init_node("ur_move", anonymous = True, disable_signals = True)
		client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
		print "Waiting for server..."
		client.wait_for_server()
		print "Connected to server"
		ur_move()
		#rospy.init_node("aimm_listener", anonymous = True)
		rospy.Subscriber("aimm_chatter", aimm_ctrl, callback)
		rospy.spin()
	except KeyboardInterrupt:
		rospy.signal_shutdown("KeyboardInterrupt")
		raise


if __name__ == '__main__': main()
		
