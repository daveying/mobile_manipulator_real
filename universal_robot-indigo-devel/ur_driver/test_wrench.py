#!/usr/bin/env python

#import roslib; roslib.load_manifest()
import math
import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32

pub_mag = rospy.Publisher('wrench_mag', Float32, queue_size = 1)

def callback(data):
	global pub_mag
	fx = data.wrench.force.x;
	fy = data.wrench.force.y;
	fz = data.wrench.force.z;
	mag = math.sqrt(fx*fx + fy*fy + fz*fz)
	mag_msg = Float32()
	mag_msg.data = mag
	pub_mag.publish(mag_msg)

def main():
	rospy.init_node("calc_mag")
	rospy.Subscriber("/wrench", WrenchStamped, callback)
	print 'hello'
	rospy.spin()

if __name__ == '__main__': main()
