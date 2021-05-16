
import rospy
import numpy as np
import keyboard
from numpy.linalg import inv
import sys
import termios
import os
import tty
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
from std_msgs.msg import ColorRGBA, Float32, Bool, Int32
from user_input.msg import Velocity, JoyCmd
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import math


msg_text = """
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

Press z to decrease speed by 20%
Press x to increase speed by 20%
___________________________________________
Press 1-5 to set the current tag
Set ready_for_arm_mov,    t = True, f = False
Set ready_for_veh_mov,    r = True, d = False
_____________________________________________
Set set_approach_retreat, e = True, s = False
Set stop_automode,        w = True, a = False


Press Shift-C to quit
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
		self.setApproachRetreatPub = rospy.Publisher('set_approach_retreat', Bool, queue_size = 1)
		self.setStopAutomodePub = rospy.Publisher('stop_automode', Bool, queue_size = 1)

		r = rospy.Rate(60)
		self.cmd_msg = JoyCmd()
		self.conn_msg = Bool()
		self.conn_msg.data = True

		self.mult = 1.0/3.0

		#Initialize state variables
		self.set_current_tag_msg = int()
		self.setReadyArm_msg = Bool()
		self.setReadyArm_msg.data = False
		self.setReadyVeh_msg = Bool()
		self.setReadyVeh_msg.data = False
		self.setApproachRetreatPub_msg = Bool()
		self.setApproachRetreatPub_msg.data = False
		self.setStopAutomodePub_msg = Bool()
		self.setStopAutomodePub_msg.data = False



		print(msg_text)



		while not rospy.is_shutdown():
			self.cmd_msg = self.detectCmds()
			# Publish at a frequency of 60 Hz
			self.joyconnPub.publish(self.conn_msg)
			self.joycmdPub.publish(self.cmd_msg)


			#Detect other inputs from the keyboard
			#Detect a commanded tag number




			r.sleep()

	def getch(self):
		fd = sys.stdin.fileno()
		old_settings = termios.tcgetattr(fd)
		try:
			tty.setraw(sys.stdin.fileno())
			ch = sys.stdin.readline(1)
			if 'C' in ch:
				sys.exit("Exiting")
			
	 
		finally:
			termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
		return ch

	def printStatus(self, str):
		sys.stdout.write("\r" + str + "                         ")
    		sys.stdout.flush()

	def detectCmds(self):
		char = self.getch()
		#print(char)
		msg = JoyCmd()
		msg.axis1 = 0
		msg.axis2 = 0
		msg.axis3 = 0


		#Non-Holonomic Steering

		#Detect forward and backward keys assigned to 'w' and 's' 
		#-> changed to 'i' and ',', (might need to the axis number was changed from 1 to 2)
		if 'i' in char:
			msg.axis2 =  1.0
		elif ',' in char:
			msg.axis2 = msg.axis2 - 1.0


		#Detect rotation keys 'q' for counterclockwise, 'e' for clockwise
		#-> changed to 'j' and 'l', (might need to be flipped -but then flipped after first test)

		elif 'l' in char:
			msg.axis3 =  1.0
		elif 'j' in char:
			msg.axis3 = msg.axis3 - 1.0


		# Holonomic Steering
		#Detect forward and backward keys assigned to 'w' and 's' 
		#-> changed to 'I' and '<', (might need to the axis number was changed from 1 to 2)
		if 'I' in char:
			msg.axis2 =  1.0
		elif '<' in char:
			msg.axis2 = msg.axis2 - 1.0

		#Detect left and right keys 'a' and 'd'
		#-> changed to 'J' and 'L'
		elif 'J' in char:
			msg.axis1 =  1.0
		elif 'L' in char:
			msg.axis1 = msg.axis1 - 1.0

		#Detect corner keys to move diagonally
		elif 'U' in char:
			msg.axis2 = 0.71
			msg.axis1 =  0.71 
		elif 'O' in char:
			msg.axis2 = 0.71
			msg.axis1 =  msg.axis1 - 0.71 
		elif '>' in char:
			msg.axis2 =  msg.axis2 - 0.71
			msg.axis1 =  msg.axis1 - 0.71 
		elif 'M' in char:
			msg.axis2 =  msg.axis2 - 0.71
			msg.axis1 =  0.71 



		#Detect keys for multipliers
		elif 'z' in char:
			self.mult = 0.8*self.mult
			self.printStatus( "currently:\tspeed %s " % (self.mult))

		elif 'x' in char:
			self.mult = 1.2*self.mult
			self.printStatus( "currently:\tspeed %s " % (self.mult))


		#Dummy character that also works to stop the vehicle
		elif 'k' in char:
			msg.axis1 = 0
			msg.axis2 = 0
			msg.axis3 = 0
			self.printStatus( "currently:\t Vehicle Stopped!")
		elif 'K' in char:
			msg.axis1 = 0
			msg.axis2 = 0
			msg.axis3 = 0
			#pass
			self.printStatus( "currently:\t Vehicle Stopped!")


		#Detect other inputs from the keyboard
		#Detect keyboard press to override the booleans
		elif 't' in char:
			self.setReadyArm_msg.data = True
			self.setReadyArmPub.publish(self.setReadyArm_msg)
			self.printStatus( "currently:\t Sent ready_for_arm_mov = True")
		elif 'f' in char:
			self.setReadyArm_msg.data = False
			self.setReadyArmPub.publish(self.setReadyArm_msg)
			self.printStatus( "currently:\t Sent ready_for_arm_mov = False")
		elif 'r' in char:
			self.setReadyVeh_msg.data = True
			self.setReadyVehPub.publish(self.setReadyVeh_msg)
			self.printStatus( "currently:\t Sent ready_for_veh_mov = True")
		elif 'd' in char:
			self.setReadyVeh_msg.data = False
			self.setReadyVehPub.publish(self.setReadyVeh_msg)
			self.printStatus( "currently:\t Sent ready_for_veh_mov = False")

		elif 'e' in char:
			self.setApproachRetreatPub_msg.data = True
			self.setApproachRetreatPub.publish(self.setApproachRetreatPub_msg)
			self.printStatus( "currently:\t Sent set_approach_retreat = True")
		elif 's' in char:
			self.setApproachRetreatPub_msg.data = False
			self.setApproachRetreatPub.publish(self.setApproachRetreatPub_msg)
			self.printStatus( "currently:\t Sent set_approach_retreat = False")
		elif 'w' in char:
			self.setStopAutomodePub_msg.data = True
			self.setStopAutomodePub.publish(self.setStopAutomodePub_msg)
			self.printStatus( "currently:\t Sent stop_automode = True")
		elif 'a' in char:
			self.setStopAutomodePub_msg.data = False
			self.setStopAutomodePub.publish(self.setStopAutomodePub_msg)
			self.printStatus( "currently:\t Sent stop_automode = False")



		else:
			# Check if one of the tag numbers was pressed
			for number in range(1,6):
				if str(number) in char:
					self.set_current_tag_msg = int(number)
					self.setTagPub.publish(self.set_current_tag_msg)
					self.printStatus( "currently:\t Sent set_current_tag = %s " % (number))	


		msg.axis1 = msg.axis1*self.mult
		msg.axis2 = msg.axis2*self.mult
		msg.axis3 = msg.axis3*self.mult

		return msg



if __name__=='__main__':
	node = keyboard_obj()
