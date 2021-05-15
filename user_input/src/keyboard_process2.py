#!/usr/bin/python


import rospy
import numpy as np
import keyboard
from numpy.linalg import inv

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
from std_msgs.msg import ColorRGBA, Float32, Bool, Int32
from user_input.msg import Velocity, JoyCmd
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import math

msg = """
Reading from the keyboard and Publishing to JoyCmd!
---------------------------
Moving forward and rotating:
        i     
   j    k    l
        ,     
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

anything else : stop

Hold down 'alt' for 1/4 speed
___________________________________________
Press 1-5 to set the current tag
Press t to set ready_for_arm_mov = True, Press f for False
Press r to set ready_for_veh_mov = True, Press d for False

CTRL-C to quit
"""

class keyboard_obj(object):

	def __init__(self):
		rospy.init_node('keyboard', anonymous=True)

		#Publishers
		self.joyconnPub= rospy.Publisher('/joy/connected', Bool, queue_size = 1)
		self.joycmdPub = rospy.Publisher('/joy/cmd', JoyCmd, queue_size = 10)
		self.setTagPub = rospy.Publisher('set_current_tag', Int32, queue_size = 1)
		self.setReadyArmPub = rospy.Publisher('ready_for_arm_mov', Bool, queue_size = 1)
		self.setReadyVehPub = rospy.Publisher('ready_for_veh_mov', Bool, queue_size = 1)


		r = rospy.Rate(60)
		self.cmd_msg = JoyCmd()
		self.conn_msg = Bool()
		self.conn_msg.data = True


		self.set_current_tag_msg = int()

		self.setReadyArm_msg = Bool()
		self.setReadyArm_msg.data = False
		self.setReadyVeh_msg = Bool()
		self.setReadyVeh_msg.data = False



		print(msg)

		while not rospy.is_shutdown():
 			
			self.cmd_msg = self.detectCmds()
			# Publish at a frequency of 60 Hz
			self.joyconnPub.publish(self.conn_msg)
			self.joycmdPub.publish(self.cmd_msg)

			#Detect other inputs from the keyboard
			#Detect a commanded tag number
			for number in range(1,6):
				if keyboard.is_pressed( str(number) ):
					self.set_current_tag_msg = int(number)
					self.setTagPub.publish(self.set_current_tag_msg)

			if keyboard.is_pressed('t'):
				self.setReadyArm_msg.data = True
				self.setReadyArmPub.publish(self.setReadyArm_msg)

			if keyboard.is_pressed('f'):
				self.setReadyArm_msg.data = False
				self.setReadyArmPub.publish(self.setReadyArm_msg)

			if keyboard.is_pressed('r'):
				self.setReadyVeh_msg.data = True
				self.setReadyVehPub.publish(self.setReadyVeh_msg)

			if keyboard.is_pressed('d'):
				self.setReadyVeh_msg.data = False
				self.setReadyVehPub.publish(self.setReadyVeh_msg)


			r.sleep()

	def detectCmds(self):
		msg = JoyCmd()

		mult = 1.0/3.0

		#Holonomic Steering
		if keyboard.is_pressed('shift'):
			#Detect forward and backward keys
			if keyboard.is_pressed('I'):
				msg.axis1 =  1.0
			if keyboard.is_pressed('<'):
				msg.axis1 = msg.axis1 - 1.0

			#Detect left and right keys 'J' and 'L'
			if keyboard.is_pressed('J'):
				msg.axis2 =  1.0
			if keyboard.is_pressed('L'):
				msg.axis2 = msg.axis2 - 1.0
	
			#Detect corner keys to move diagonally 
			if keyboard.is_pressed('U'):
				msg.axis1 =  0.71
				msg.axis2 =  0.71
			if keyboard.is_pressed('O'):
				msg.axis1 = 0.71
				msg.axis2 = msg.axis2 - 0.71
			if keyboard.is_pressed('>'):
				msg.axis1 = msg.axis1 - 0.71
				msg.axis2 = msg.axis2 - 0.71
			if keyboard.is_pressed('M'):
				msg.axis1 = 0.71
				msg.axis2 = msg.axis2 - 0.71
		
		else:
			#Non-Holonomic Steering
			#Detect forward and backward keys
		
			if keyboard.is_pressed('i'):
				msg.axis1 =  1.0
			if keyboard.is_pressed(','):
				msg.axis1 = msg.axis1 - 1.0

			#Detect rotation keys 'j' for counterclockwise, 'l' for clockwise

			if keyboard.is_pressed('j'):
				msg.axis3 =  1.0
			if keyboard.is_pressed('l'):
				msg.axis3 = msg.axis3 - 1.0

		#Detect keys for multipliers

		if keyboard.is_pressed('alt'):
			mult = 0.25 * mult
			
		#if keyboard.is_pressed('shift'):
		#	mult = 1.0

		msg.axis1 = msg.axis1*mult
		msg.axis2 = msg.axis2*mult
		msg.axis3 = msg.axis3*mult
		return msg





if __name__=='__main__':
	node = keyboard_obj()
	


# var joy_connected_topic = new ROSLIB.Topic({
#     ros: ros,
#     name: '/joy/connected',
#     messageType: 'std_msgs/Bool'
# })
# var joy_cmd_topic = new ROSLIB.Topic({
#     ros: ros,
#     name: '/joy/cmd',
#     messageType: 'user_input/JoyCmd'
# })
