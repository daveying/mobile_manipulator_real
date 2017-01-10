##! /usr/bin/env python
# Filename: stop_agv.py
# Author: Dave

import socket

import rospy
from std_msgs.msg import Empty

agv_ctrl_cmd_head = '%c%c%c' % (250, 251, 1) #'FAFB01'
agv_ctrl_cmd_tail = '%c%c' % (238, 255) #'EEFF'

sock = None
addr = None

def callback(data):
	global sock, addr
	agv_ctrl_cmd = agv_ctrl_cmd_head + ('%c%c' % (0, 0)) + ('%c%c' % (0, 0)) + ('%c' % 0) + agv_ctrl_cmd_tail
	sock.send(agv_ctrl_cmd)

def main():
	global sock, addr
	try:
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.bind(('192.168.11.107', 12345))
		s.listen(5)
		sock, addr = s.accept()
		
		rospy.init_node('agv_stop', anonymous = True)
		rospy.Subscriber('agv_stop', Empty, callback)
		rospy.spin()
	except KeyboardInterrupt:
		s.close()
		raise

if __name__ == '__main__': main()
