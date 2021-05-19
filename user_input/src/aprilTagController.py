#!/usr/bin/env python

# 2.12 Final Project
# Phillip Daniel April 2021

# Modifications from Ava, 5/15/2021
# Modifications from Tyler 5/15/2021

import rospy
import numpy as np
import math

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from std_msgs.msg import Bool, Int32

import serial
from apriltag_ros.msg import AprilTagDetectionArray
from user_input.msg import Velocity, JoyCmd

################################################################
# Define Global Variables
# These global variables are the pose of the mobile robot
x = 0.0
y = 0.0
theta 	= 0.0
speed = 0.5

# Tag information
tagPose = None
tagID = None

# Approach and retract distances
tagAppDists = [0,0.5,0.5,0.5,1.22,0.27]      # index of list corresponds to tag ID, entry corresponds to approach distance
tagRetDists = [0,0,0,2.785,3.965,1.295] # index of list corresponds to tag ID, entry corresponds to approach distance
tagAppDist  = 0.6 # initialize to something large to start
tagRetDist  = 5.0  # initialize to something large to start

# Target tags, control booleans, etc. - the node subscribes to this information.
target 	= None     # Target marker id
appret 	= True    # Approach return boolean 
stopAuto = False   # Stop autonomous mode

#################################################################


def euler_from_quaternion(x, y, z, w):
        """
	From https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/        
	Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


def pointAtTag(targetTagID):
	# Description: Search for the specified tag
	# Return
		# void - Prints a message when we are pointing at a tag
	# Argument
		# targetTagID - The ID of the tag that we wish to point the robot's camera towards
	print("starting to point at target")
	zDot=0
	xDot=0
	rate=.5
	pointed = False
	
	jcv = JoyCmd()
	jcv.axis1 = 0.0
	jcv.axis2 = 0.0
	jcv.btn1 = 0.0
	jcv.btn2 = 0.0
	jcv.btn3 = 0.0


	while pointed == False:
		if rospy.is_shutdown():
			break

		# Spin around and look for the tag 'locatedTag.label.' Stop once we are pointing at it within a small window for error
		if targetTagID == tagID and tagPose != None: # We have found the tag that we were looking for.
			relX=tagPose.pose.pose.position.x
			print('tag position = ', relX)
			if relX<.1 and relX>-.1: # If we are pointing at the tag  -> original values were .05
				thetaDot=0
				pointed = True
			else:
				pointed = False
		else:
			thetaDot=rate
			pointed = False

		jcv.axis3 = thetaDot
		virtualJoy_pub.publish(jcv)
	print('Pointed at tag = ', tagID)



def viewedTagRelPos(data):
	# Description: Relative position of the tag 'tagID,' if it is in view
		# Return
			# locatedTag.label - The label for the visible April tags (e.g. 'tag1')
			# locatedTag.relZ - The x-axis position of the April tag with respect to the coordinate system
			#		    that is attached to the mobile robot
			# locatedTag.relX - The y-axis position of the April tag with respect to the coordinate system 
			#		    that is attached to the mobile robot
		# Argument
			# tagID - The ID of the tag that we wish to point the robot's camera towards

	##  Need to also include the 'gobal' declaration here too since this is a callback.
	global tagPose
	global tagID

	tagPose = None
	tagID = None

	for detections in data.detections:
		#print('Detection IDs', detections.id)
		for idseen in detections.id:
			if idseen == 0 or idseen == 1 or idseen == 2 or idseen == 3 or idseen == 4 or idseen == 5 :
				tagID = idseen
				tagPose = detections.pose

				## Also need to include the 'global' declaration here too.  Damn this is confusing.
				global tagAppDist
				global tagRetDist

				tagAppDist = tagAppDists[(idseen)] # Assign tag approach distance based on the tag you're looking at
				tagRetDist = tagRetDists[(idseen)]
				#print('tag', tagID, 'detected')
				#print('x', tagPose.pose.pose.position.x)
				#print('z', tagPose.pose.pose.position.z)
			else: 
				print('Check for pose of tags')

def setTarget(data):
	# include 'global' since this is a callback
	global target
	target = data.data

def setApproach(data):
	# include 'global' since this is a callback
	global appret
	appret = data.data

def stopAuto(data):
	# include 'global' since this is a callback
	global stopAuto
	stopAuto = data.data


def approachRetreat():
	# Description: Drive within a certain distace of the tag 'targetTagID'
		# Return
			# null
		# Argument
			# targetTagID - The ID of the tag that we wish to point the robot's camera towards

	#relX=tagPose.pose.pose.position.x
	#relZ=tagPose.pose.pose.position.z
	approached=False
	
	jcv = JoyCmd()
	jcv.axis3 = 0.0
	jcv.btn1 = 0.0
	jcv.btn2 = 0.0
	jcv.btn3 = 0.0
	
	while approached==False:			

		if rospy.is_shutdown():
			break

		if tagPose != None:
			relX=tagPose.pose.pose.position.x
			relZ=tagPose.pose.pose.position.z
			q_x = tagPose.pose.pose.orientation.x
			q_y = tagPose.pose.pose.orientation.y
			q_z = tagPose.pose.pose.orientation.z
			q_w = tagPose.pose.pose.orientation.w
			relRollX, relPitchY, relYawZ = euler_from_quaternion(q_x, q_y, q_z, q_w)

			xDot = 0.0
			zDot = 0.0
			thetaDot = 0.0			

			
			print('Approaching x=', relX)
			print('Approaching z=', relZ)
			print('Approaching relYawZ =', relYawZ)
			print('Approaching relRollX =', relRollX)
			print('Approaching relPitchY =', relPitchY)

			# Error criteria for z distance away is 'tagAppDist'
			# Error criteria for x distance away is hard coded below
			xMov = 0.0
			zMov = 0.0
			yawMov = 0.0

			# Set the x, z, and theta velocities
			if abs(relX) > .01:
				# keep moving in the x (sideways) until error is small
				xMov = relX
				
			#  Adjust the zMoz depending on if we are approaching or retreating
			if ( (abs(relZ) > tagAppDist) and appret ):
				# keep moving in the z (forwards) until within range
				zMov = relZ
				print("Approaching tag ", tagID)

			if ( (abs(relZ) < tagRetDist) and (not appret ) ):
				# keep moving in the negative z direction until within range
				error = -1.0 *( tagRetDist - relZ) -.3  # This should always be a negative number
				zMov =  error
				print("Retreating from tag ", tagID)


			if abs(relPitchY) > .05:
				# keep rotating about towards the center of the tag
				yawMov = relPitchY


			# normalize the commands
			vRel=np.array([xMov,zMov])
			relPosNorm=np.linalg.norm(vRel) # This relative position vector only involves X,Z, 
							# not orientation (assumes X within range based on line 59
			
			if relPosNorm > 0:
				# we need to move
				# Calculate speed and cap it
				speed = 0.2* relPosNorm
				if speed > .5:
					speed = .5

				xDot = -xMov / relPosNorm * speed
				zDot = zMov / relPosNorm * speed

			if yawMov > 0:
				# we need to rotate
				thetaDot = -yawMov * .1 # hard code for now


			if (relPosNorm == 0.0) and (yawMov == 0.0):
				# We have arrived - both translation and rotation errors are within limits	
				xDot=0			
				zDot=0
				thetaDot = 0.0
				approached=True
				at_target_pub.publish(tagID)

		else:
			zDot=0
			xDot=0
			print 'lost tag'

		jcv.axis1 = xDot
		jcv.axis2 = zDot
		jcv.axis3 = thetaDot
		print( "Sending motor commands: Axis1 = ", xDot, " Axis2 = ", zDot, " Axis3 = ", thetaDot)
		virtualJoy_pub.publish(jcv)
		r.sleep()
	print('Arrived at Position')



#############################################################################
#############################################################################
## Start of main code

#initialize node
rospy.init_node('TagChaser', anonymous=True)

# Subscribers
apriltag_sub 		= rospy.Subscriber("/tag_detections", AprilTagDetectionArray, viewedTagRelPos)
drive_to_this_tag_sub 	= rospy.Subscriber("/drive_to_this_tag", Int32, setTarget)
set_current_tag_sub 	= rospy.Subscriber("/set_current_tag", Int32, setTarget) # for testing
approach_retreat_sub 	= rospy.Subscriber("/approach_retreat", Bool, setApproach) # True=approach
stop_automode 		= rospy.Subscriber("/stop_automode", Bool, stopAuto)
print("Subscriber setup")

# Publishers
virtualJoy_pub 		= rospy.Publisher("/joy/cmd", JoyCmd, queue_size=1)
at_target_pub 		= rospy.Publisher("/at_target", Int32, queue_size=1)
print("Publisher setup")


r = rospy.Rate(50)
print("ROS rate setup")


appret = True # set for testing
stopAuto = False # set for testing

# This is the main loop
while not rospy.is_shutdown():	

	target = 1 #Hardcode to whatever tag is in video feed for testing
	if not stopAuto:
		print("Start aprilTagController")
		pointAtTag(target)
		approachRetreat()
		print 'Done'
		
	r.sleep()

# Finally, stop the robot when shutting down
jcv = JoyCmd()
jcv.axis1 = 0.0
jcv.axis2 = 0.0
jcv.axis3 = 0.0
jcv.btn1 = 0.0
jcv.btn2 = 0.0
jcv.btn3 = 0.0
virtualJoy_pub.publish(jcv)
