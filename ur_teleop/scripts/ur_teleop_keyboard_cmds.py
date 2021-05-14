#!/usr/bin/python


import rospy
import numpy as np
import keyboard
from numpy.linalg import inv
import sys
import termios
import os
import tty
#from sensor_msgs.msg import Image, CameraInfo
#from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
from std_msgs.msg import ColorRGBA, Float32, Bool, Int32
#from user_input.msg import Velocity, JoyCmd
#from cv_bridge import CvBridge, CvBridgeError
#import message_filters
import math

class keyboard_obj(object):

	def __init__(self):
		rospy.init_node('ur_teleop_keyboard', anonymous=True)
		self.stopPub = rospy.Publisher('/arm/stop_arm', Bool, queue_size = 1)
                r = rospy.Rate(60)
		self.stop_msg = Bool()
		self.stop_msg.data = False
		while not rospy.is_shutdown():
			self.stop_msg = self.detectCmds_stop()
			self.stopPub.publish(self.stop_msg)
			r.sleep()

	def getch(self):
		fd = sys.stdin.fileno()
		old_settings = termios.tcgetattr(fd)
		try:
			tty.setraw(sys.stdin.fileno())
			ch = sys.stdin.readline(1)
	 
		finally:
			termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
		return ch

	def detectCmds_stop(self):
		char = self.getch()
		print(char)
		msg = Bool()

		if 's' in char: # 's' for stop
			msg.data = True
		elif 'g' in char: # 'g' for go
			msg.data = False
		return msg


if __name__=='__main__':
	node = keyboard_obj()
	
