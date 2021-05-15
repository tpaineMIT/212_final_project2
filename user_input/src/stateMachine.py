#!/usr/bin/env python

# 2.12 Final Project
# Phillip Daniel April 2021

# Modifications from Ava, 5/15/2021

import rospy
import numpy as np

import serial
from apriltag_ros.msg import AprilTagDetectionArray
from user_input.msg import Velocity, JoyCmd
from std_msgs.mst import Bool, Int32

# global variables
appret = True
atTarget = 0
setTarget = 1
readyVeh = False # Wait for user to update ready

def atTarget(data):
	atTarget = data
def setAppRet(data):
	appret = data
def setTarget(data):
	setTarget = data
def readyVeh(data):
	readyVeh = data
def viewedTagID(data):
	global viewedTag
	viewedTag = data.detections.id[0] # Assumes only one tag in field of view

rospy.init_node('StateMachine', anonymous=True)

drive_to_this_tag_pub = rospy.Publisher("/drive_to_this_tag", Int32, queue_size=1)
approach_retreat_pub = rospy.Publisher("/approach_retreat", Bool, queue_size=1) # True=approach
ready_for_arm_mov_pub = rospy.Publisher("/ready_for_arm_mov", Bool, queue_size=1)

apriltag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, viewedTagID)
ready_for_veh_mov_sub = rospy.Subscriber("/ready_for_veh_mov", Bool, readyVeh)
set_approach_retreat_sub = rospy.Subscriber("/set_approach_retreat", Bool, setAppRet)
set_current_tag_sub = rospy.Subscriber("/set_current_tag", Int32, setTarget)
at_target_sub = rospy.Subscriber("/at_target", Int32, atTarget)

r = rospy.Rate(50)
print("ROS rate setup")

# This is the main loop
while not rospy.is_shutdown(): # need to include user interrupt? maybe user interrupt is just setting readyVeh=False?
	if readyVeh:
		if viewedTag == 5 and atTarget==5 and appret:
			ready_for_arm_mov_pub.publish(True)
			#appret = False
		else:
			approach_retreat_pub.publish(appret)
			drive_to_this_tag_pub.publish(setTarget)
			if atTarget==setTarget:
				if appret and setTarget<5: # maybe this is redundant with the if
					setTarget++
				elif setTarget >= 3 and not appret:
					# do something

	if readyVeh:
		while True: # assumes appret=True?
			# block of code: approach retreat publish, drive to the tag, hear that we've arrived at the tag, increment the tag, repeat until condition
			if viewedTag == 5 and atTarget==5 and appret:
				break # not sure we really want to break though, just want to switch to waiting for arm team and put in retreat mode
		# wait for readyVeh=True --> don't need coded since if readyVeh wrapper, and put system in retreat mode
		# do a similar do-while where the block of code is retreating and the condition is we've arrived at the final retreat target

	# when set_current_tag changes, start the sequence from the received tag
	# need to increment tag to pass to aprilTagController
	# once goes 1-5, update ready for arm movement
	# listen for ready for veh movement
	# go into retreat sequence

	r = rospy.Rate(1)
	while not rospy.is_shutdown():
		r.sleep()

