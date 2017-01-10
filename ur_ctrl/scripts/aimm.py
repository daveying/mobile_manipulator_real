##! /usr/bin/env python
# Filename: aimm.py
# Author: Dave

import socket
import threading
import time
import binascii
import numpy

import roslib; roslib.load_manifest('ur_driver')
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *

#import roslib; roslib.load_manifest('')
import rospy
from geometry_msgs.msg import Twist #type of /turtle1/cmd_vel topic
from std_msgs.msg import Empty
from std_msgs.msg import Int16

agv_ctrl_cmd_head ='%c%c%c' % (250, 251, 1) #'FAFB01'
agv_ctrl_cmd_tail ='%c%c' % (238, 255) #'EEFF'
rLock = threading.RLock()
x = 0
y = 0
a = 0

sock = None
addr = None
client = None
pub = None

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
q1=[-2.703,-1.89,-1.76,-2.57,0.32,2.21]
q2=[-1.25,-1.89,-1.76,-2.57,0.32,2.21]
q3=[-1.25,-2.12,-1.89,-2.2,0.32,2.21]
q4=[-1.4,-2.2,-1.89,-2.06,0.18,2.15]
t1 = 8
t2 = 16
t3 = 17
t4 = 18


def tohex(val, nbits):
	return hex((val + (1 << nbits)) % (1 << nbits))

def tcplink(sock, addr):
	try:
		global x, y, a
		rLock.acquire()
		print 'x:: %d;y:: %d' % (x, y)
		xs = tohex(x, 16)
		ys = tohex(y, 16)
		print xs + ys
		xlen = xs.__len__()
		ylen = ys.__len__()
		xtemp = ''
		ytemp = ''
		if xlen != 6:
			for i in range(6 - xlen):
				xtemp += '0'
			xtemp += xs[2:]
		else:
			xtemp = xs[2:]
		if ylen != 6:
			for i in range(6 - ylen):
				ytemp += '0'
			ytemp += ys[2:]
		else:
			ytemp = ys[2:]
		x1 = binascii.a2b_hex(xtemp[0:2]);print 'x1: ' + xtemp[0:2]
		x2 = binascii.a2b_hex(xtemp[2:4]);print 'x2: ' + xtemp[2:4]
		y1 = binascii.a2b_hex(ytemp[0:2]);print 'y1: ' + ytemp[0:2]
		y2 = binascii.a2b_hex(ytemp[2:4]);print 'y2: ' + ytemp[2:4]
		agv_ctrl_cmd = agv_ctrl_cmd_head + ('%c%c' % (x1, x2)) + ('%c%c' % (y1, y2)) + ('%c' % int(a*10)) + agv_ctrl_cmd_tail
		print agv_ctrl_cmd
		print 'x_ = %d; y_ = %d' % (x, y)
		#print 'xx = %d; yy = %d' % (x1*0xff+x2, y1*0xff+y2)
		#x = 0
		#y = 0
		#a = 0
		rLock.release()
		sock.send(agv_ctrl_cmd)
	except KeyboardInterrupt:
		agv_ctrl_cmd = agv_ctrl_cmd_head + ('%c%c' % (0, 0)) + ('%c%c' % (0, 0)) + ('%c' % 0) + agv_ctrl_cmd_tail
		sock.send(agv_ctrl_cmd)
		raise

def keyboardCB(data):
	global x, y, a, sock, addr, i
	rLock.acquire()
	#x = data.linear.x
	#y = data.angular.z
	if data.linear.x > 0:
		y += 10
	if data.linear.x < 0:
		y -= 10
	if data.angular.z >= 0:
		x -= 10
	if data.angular.z <= 0:
		x += 10
	
	print 'x = %d; y = %d' % (x, y)
	rLock.release()
	t = threading.Thread(target = tcplink, args = (sock, addr))
	t.start()

def pickUpCB(data):
	global client
	g = FollowJointTrajectoryGoal()
	g.trajectory = JointTrajectory()
	g.trajectory.joint_names = JOINT_NAMES
	g.trajectory.points = [JointTrajectoryPoint(positions = q1, velocities = [0] * 6, time_from_start = rospy.Duration(t1)), JointTrajectoryPoint(positions = q2, velocities = [0] * 6, time_from_start = rospy.Duration(t2)), JointTrajectoryPoint(positions = q3, velocities = [0] * 6, time_from_start = rospy.Duration(t3)), JointTrajectoryPoint(positions = q4, velocities = [0] * 6, time_from_start = rospy.Duration(t4))]
	client.send_goal(g)
	try:
		client.wait_for_result()
	except KeyboardInterrupt:
		client.cancel_goal()
		raise
	pub.publish(140)  #gripper close
	g = FollowJointTrajectoryGoal()
	g.trajectory = JointTrajectory()
	g.trajectory.joint_names = JOINT_NAMES
	g.trajectory.points = []
	g.trajectory.points = [JointTrajectoryPoint(positions = q3, velocities = [0] * 6, time_from_start = rospy.Duration(1)), JointTrajectoryPoint(positions = q2, velocities = [0] * 6, time_from_start = rospy.Duration(2)), JointTrajectoryPoint(positions = q1, velocities = [0] * 6, time_from_start = rospy.Duration(8))]
	client.send_goal(g)
	try:
		client.wait_for_result()
	except KeyboardInterrupt:
		client.cancel_goal()
		raise
	
def pickDownCB(data):
	global client
	g = FollowJointTrajectoryGoal()
	g.trajectory = JointTrajectory()
	g.trajectory.joint_names = JOINT_NAMES
	g.trajectory.points = [JointTrajectoryPoint(positions = q2, velocities = [0] * 6, time_from_start = rospy.Duration(t2)), JointTrajectoryPoint(positions = q3, velocities = [0] * 6, time_from_start = rospy.Duration(t3)), JointTrajectoryPoint(positions = q4, velocities = [0] * 6, time_from_start = rospy.Duration(t4))]
	client.send_goal(g)
	try:
		client.wait_for_result()
	except KeyboardInterrupt:
		client.cancel_goal()
		raise	
	#pub = rospy.Publisher('toggle_led', Int16, queue_size = 2)
	pub.publish(210)  #gripper open
	g.trajectory.points = []
	g.trajectory.points = [JointTrajectoryPoint(positions = q3, velocities = [0] * 6, time_from_start = rospy.Duration(1)), JointTrajectoryPoint(positions = q2, velocities = [0] * 6, time_from_start = rospy.Duration(2)), JointTrajectoryPoint(positions = q1, velocities = [0] * 6, time_from_start = rospy.Duration(8))]
	client.send_goal(g)
	try:
		client.wait_for_result()
	except KeyboardInterrupt:
		client.cancel_goal()
		raise

##def displaceCB(data):
##	data.x	

def main():
	global sock, addr, client, pub
	try:
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.bind(('192.168.11.107', 12345))
		#s.bind(('172.17.5.31', 12345))
		s.listen(5)
		sock, addr = s.accept()
		
		pub = rospy.Publisher('toggle_led', Int16, queue_size = 2)
		rospy.init_node('aimm_ctrl_interface', anonymous = True)
		pub.publish(210) #gripper 
		rospy.Subscriber('turtle1/cmd_vel', Twist, keyboardCB)
		##rospy.Subscriber('agv_displacement', Displacement, displaceCB)
		client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
		print "Waiting for follow_joint_trajectory server..."
		#client.wait_for_server()
		print "Connected to server"
		rospy.Subscriber('pick_up', Empty, pickUpCB)
		rospy.Subscriber('pick_down', Empty, pickDownCB)
		rospy.spin()
	except KeyboardInterrupt:
		s.close()
		raise	

if __name__ == '__main__': main()

