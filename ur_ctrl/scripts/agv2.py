##! /usr/bin/env python
# Filename: agv2.py
# Author: Dave


import socket
import threading
import time
import binascii
import numpy

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

agv_ctrl_cmd_head = '%c%c%c' % (250, 251, 1) #'FAFB01'
agv_ctrl_cmd_tail = '%c%c' % (238, 255) #'EEFF'
vx = 0
vy = 0
wz = 0

sock = None
addr = None

stop = False #when stop msg coming, stop = True, and just stop command will transmitting

def tohex(val, nbits):
	return hex((val+(1<<nbits))%(1<<nbits))

def callback2(data):
	global sock, addr, stop
	stop = True
	agv_ctrl_cmd = agv_ctrl_cmd_head + ('%c%c' % (0, 0)) + ('%c%c' % (0, 0)) + ('%c' % 0) + agv_ctrl_cmd_tail
	sock.send(agv_ctrl_cmd)
	print 'agv_stopped'

def callback(data):
	global vx,vy,wz,sock,addr,stop
	if data.linear.x > 300: ##!!!!!!!!
		data.linear.x = 300
	if data.linear.x < -300:
		data.linear.x = -300
	if data.linear.y > 300:
		data.linear.y = 300
	if data.linear.y < -300:
		data.linear.y = -300
#	if data.angular.z > 20:
#		data.angular.z = 20
#	if data.angular.z < -20:
#		data.angular.z = -20
	vx = int(round(data.linear.x))
	print 'vx = '+vx.__str__()
	vy = int(round(data.linear.y))
	print 'vy = '+vy.__str__()
#	wz = int(round(data.angular.z))
#	if wz < 0:
#		wz = -wz
	wz=0  #angular velocity always set to be zero
	print 'wz = '+wz.__str__()
	if not stop:
		vxs = tohex(vx, 16)
		vys = tohex(vy, 16)
		wzs = tohex(wz, 16)

		xlen = vxs.__len__()
		ylen = vys.__len__()
		zlen = wzs.__len__()

		xtemp = ''
		ytemp = ''
		ztemp = ''

		if xlen != 6:
			for i in range(6 - xlen):
				xtemp += '0'
			xtemp += vxs[2:]
		else:
			xtemp = vxs[2:]

		if ylen != 6:
			for i in range(6 - ylen):
				ytemp += '0'
			ytemp += vys[2:]
		else:
			ytemp = vys[2:]

		if zlen != 6:
			for i in range(6 - zlen):
				ztemp += '0'
			ztemp += wzs[2:]
		else:
			ztemp = wzs[2:]
	
		vx1 = binascii.a2b_hex(xtemp[0:2])
		vx2 = binascii.a2b_hex(xtemp[2:4])
		vy1 = binascii.a2b_hex(ytemp[0:2])
		vy2 = binascii.a2b_hex(ytemp[2:4])
		wz1 = binascii.a2b_hex(ztemp[0:2])#useless
		wz2 = binascii.a2b_hex(ztemp[2:4])
		#wz2 = int(wz2/10)

		agv_ctrl_cmd = agv_ctrl_cmd_head + ('%c%c' % (vx1, vx2)) + ('%c%c' % (vy1, vy2)) + ('%c' % wz2) + agv_ctrl_cmd_tail
		sock.send(agv_ctrl_cmd)
		cmd_str = '0'
		for i in agv_ctrl_cmd:
			cmd_str += hex(ord(i)).__str__()
			cmd_str += ', '
		print cmd_str + '\n'
		
def main():
	global sock, addr
	try:
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.bind(('192.168.11.107', 12345))
		s.listen(5)
		sock, addr = s.accept()
		
		rospy.init_node('agv_ctrl_interface', anonymous = True)
		rospy.Subscriber('/robot_0/cmd_vel', Twist, callback)
		rospy.Subscriber('agv_stop', Empty, callback2)
		rospy.spin()
	except KeyboardInterrupt:
		s.close()
		raise	

if __name__ == '__main__': main()


