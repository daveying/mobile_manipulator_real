#!/usr/bin/env python

#from beginner_tutorials.srv import *
from agile_grasp2.srv import hksGripper as hksGripper

import rospy
import socket
import string


def hks_gripper_srv_main():
        rospy.wait_for_service('hksGpripperSrv2')
        add_two_ints = rospy.ServiceProxy('hksGpripperSrv2', hksGripper)
        ret=add_two_ints(120)
	print ret


if __name__ == "__main__":
	hks_gripper_srv_main()
	
