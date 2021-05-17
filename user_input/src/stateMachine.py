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

########################################################################
# global variables
appret = True
atTarget = 0
target = 1
readyVeh = False # Wait for user to update ready

########################################################################

def atTarget(msg):
	# include 'global' since it is in a callback
	global atTarget
	atTarget = msg.data

def setAppRet(msg):
	# include 'global' since it is in a callback
	global appret
	appret = msg.data

def setTarget(msg):
	# include 'global' since it is in a callback
	global setTarget
	target = msg.data

def readyVeh(msg):
	# include 'global' since it is in a callback
	global readyVeh
	readyVeh = msg.data

def viewedTagID(data):
	# include 'global' since it is in a callback
	global viewedTag
	viewedTag = data.detections.id[0] # Assumes only one tag in field of view

################################################################################
### Initialize ros node and advertise sub and pubs

rospy.init_node('StateMachine', anonymous=True)

# Publishers 
drive_to_this_tag_pub	 = rospy.Publisher("/drive_to_this_tag", Int32, queue_size=1)
approach_retreat_pub	 = rospy.Publisher("/approach_retreat", Bool, queue_size=1) # True=approach
ready_for_arm_mov_pub	 = rospy.Publisher("/ready_for_arm_mov", Bool, queue_size=1)

# Subscribers 
apriltag_sub 		 = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, viewedTagID)
ready_for_veh_mov_sub	 = rospy.Subscriber("/ready_for_veh_mov", Bool, readyVeh)
set_approach_retreat_sub = rospy.Subscriber("/set_approach_retreat", Bool, setAppRet)
set_current_tag_sub	 = rospy.Subscriber("/set_current_tag", Int32, setTarget)
at_target_sub 		 = rospy.Subscriber("/at_target", Int32, atTarget)

r = rospy.Rate(50)
print("ROS rate setup")


############################################################################
# This is the main loop
while not rospy.is_shutdown():
# Potential for user interrupt: set readyVeh to False until changes are made (maybe any keypress in teleop while readyVeh=True automatically switches readyVeh to False?) Then user can adjust /set_current_tag, /set_approach_retreat, or reset /at_target if necessary

# Logic using if statements
	if readyVeh:
		# If arrived at docking tag in the approach phase, set arm movement to True, appret to False, and wait for readyVeh to be True again
		if viewedTag==5 and atTarget==5 and appret:
			ready_for_arm_mov_pub.publish(True)
			appret = False
			readyVeh = False # Need arm team to publish True to /ready_for_veh_mov to start process again
		# If finished retreating, stop our movement
		elif viewedTag==3 and atTarget==3 and not appret:
			print("Finished!")
			readyVeh = False # essentially stop our movement. Assumes arm or teleop isn't continually publishing "True"
		# Otherwise, we need movement
		else:
			# Send movement directions to aprilTagController.py
			approach_retreat_pub.publish(appret) # Approach or retreat?
			drive_to_this_tag_pub.publish(target) # Which tag to approach/retreat to/from?
			
			# When we've arrived at the target, increment target according to whether we're approaching or retreating
			if atTarget==target:
				# If approaching, increment target by 1
				if appret and target<5: # need to include and target<5 to move successful docking into next loop to be caught by outer if statement
					target++
				elif appret and target==5: # arrived at docking so want to move to next time step through loop
					break
				# If retreating and not yet reached finish line, decrement target by 1
				elif target>3 and not appret:
					target--
				# Otherwise, successfully docked or finished and want to move to next step in loop

			# Otherwise, atTarget != target so approach/retreat failed / not yet complete. Want to continue to next step in loop, so do no action where the else would be


# Same logic but in while loop format
	if readyVeh:
		while appret and not (viewedTag==5 and atTarget==5): # While we need movement in approach
			# Send movement directions to aprilTagController.py
			approach_retreat_pub.publish(appret) # Approach or retreat?
			drive_to_this_tag_pub.publish(target) # Which tag to approach/retreat to/from?
			if atTarget==target:
				target++
		# If while loop is over, then we must have successfully docked
		ready_for_arm_mov_pub.publish(True)
		appret = False
		readyVeh = False # wait for arm team or user to publish /ready_for_veh_mov to True again
		
		while not appret and not (viewedTag==3 and atTarget==3): # While we need movement in retreat
			# Send movement directions to aprilTagController.py
			approach_retreat_pub.publish(appret) # Approach or retreat?
			drive_to_this_tag_pub.publish(target) # Which tag to approach/retreat to/from?
			if atTarget==target:
				target--
		# If while loop is over, then we must have successfully retreated


	r = rospy.Rate(1)
	while not rospy.is_shutdown():
		r.sleep()

