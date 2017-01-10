##! /usr/bin/env python
# Filename: agv.py
# Author: Dave

import socket
import threading
import time
import binascii
import numpy

#import roslib; roslib.load_manifest('')
import rospy
from geometry_msgs.msg import Twist #type of /turtle1/cmd_vel topic

agv_ctrl_cmd_head ='%c%c%c' % (250, 251, 1) #'FAFB01'
agv_ctrl_cmd_tail ='%c%c' % (238, 255) #'EEFF'
rLock = threading.RLock()
x = 0
y = 0
a = 0

sock = None
addr = None

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

def callback(data):
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
	

def main():
	global sock, addr
	try:
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.bind(('192.168.11.107', 12345))
		s.listen(5)
		sock, addr = s.accept()
		
		rospy.init_node('agv_ctrl_interface', anonymous = True)
		rospy.Subscriber('turtle1/cmd_vel', Twist, callback)
		rospy.spin()
	except KeyboardInterrupt:
		s.close()
		raise	

if __name__ == '__main__': main()

