#! /usr/bin/env python
# Filename: agv_with_stm32.py
# Author: Dave

import serial
import binascii
import struct
import threading
import time

class AGV(object):
	def __init__(self):
		self.is_setting = False
		self.vx = 0  #unit is mm/s
		self.vy = 0
		self.w = 0   #unit is degree/s
		self.setting_flag_lock = threading.RLock()
		print 'Initializing serial port...'
		try:
			self._serl = serial.Serial('/dev/ttyUSB0',115200)
			time.sleep(1.5)
			print 'Serial is ok!'
		except Exception, e:
			print e.__str__()

	def send_velocities(self):
		cmd_str = struct.pack('>BBBhhbBBBB', 0xfa, 0xfb, 01, int(round(self.vx)), int(round(self.vy)), int(round(self.w)), 0xee, 0xff, 0x3f, 0x3b)
		cmd = bytearray(cmd_str)
		cmdd = ''		
		for i in cmd:
			cmdd+= hex(i).__str__()
			cmdd+=' '
		print cmdd
		n = self._serl.write(cmd)
		if n==len(cmd):
			echo = self._serl.read(n)
			print type(echo)
			eee = ''
			for ii in echo:
				
				eee += hex(ord(ii)).__str__()
				eee += ' '
			print 'echo: ' + eee
			print 'sending velocities success: vx: '+self.vx.__str__()+' vy:'+self.vy.__str__()+' w:'+self.w.__str__()
		else:
			print 'sending velocities failure...'
	def reset(self):
		cmd_str = struct.pack('>BBBhhbBBBB', 0xfa, 0xfb, 00, 00, 00, 00, 0xee, 0xff, 0x3f, 0x3b)
		cmd = bytearray(cmd_str)
		n = self._serl.write(cmd)
		time.sleep(0.2)
		n = self._serl.write(cmd)
		if n == len(cmd):
			print 'reset success.'
		else:
			print 'reset failure...'

	def set_velocities(self, _vx, _vy, _w):
#		if _vx > 1000:
#			_vx = 1000
#		if _vx < -1000:
#			_vx = -1000
#		if _vy > 1000:
#			_vy = 1000
#		if _vy < -1000:
#			_vy = -1000
#		if _w > 20:
#			_w = 20
#		if _w < -20:
#			_w = -20


		self.vx = _vx
		self.vy = _vy
		self.w = _w

	def close_com(self):
		self._serl.close()


