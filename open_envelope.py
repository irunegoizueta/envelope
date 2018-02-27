#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import math
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose

import roslib; roslib.load_manifest('robotiq_s_model_control')
from robotiq_s_model_control.msg import _SModel_robot_output  as outputMsg
from time import sleep

from std_msgs.msg import String
from robotiq_force_torque_sensor.msg import ft_sensor ##anadido para force sensor
from robotiq_force_torque_sensor.srv import sensor_accessor ##anadido para force sensor
from geometry_msgs.msg import WrenchStamped
from rq_test_sensor import ForceSensor

Force = ForceSensor()

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_ur10_robot',
                anonymous=True)

## Instantiate a RobotCommander object.  This object is an interface to
## the robot as a whole.
robot = moveit_commander.RobotCommander()

## Instantiate a PlanningSceneInterface object.  This object is an interface
## to the world surrounding the robot.
scene = moveit_commander.PlanningSceneInterface()

## Instantiate a MoveGroupCommander object.  This object is an interface
## to one group of joints. 
group = moveit_commander.MoveGroupCommander("manipulator")
  
display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)

## Function to control the speed of the movement
def scale_trajectory_speed(traj, scale):
       # Create a new trajectory object
       new_traj = RobotTrajectory()
      
       # Initialize the new trajectory to be the same as the planned trajectory
       new_traj.joint_trajectory = traj.joint_trajectory
      
       # Get the number of joints involved
       n_joints = len(traj.joint_trajectory.joint_names)
      
       # Get the number of points on the trajectory
       n_points = len(traj.joint_trajectory.points)
       
       # Store the trajectory points
       points = list(traj.joint_trajectory.points)

       # Cycle through all points and scale the time from start, speed and acceleration
       for i in range(n_points):
           point = JointTrajectoryPoint()
           point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale
           point.velocities = list(traj.joint_trajectory.points[i].velocities)
           point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
           point.positions = traj.joint_trajectory.points[i].positions
                         
           for j in range(n_joints):
               point.velocities[j] = point.velocities[j] * scale
               point.accelerations[j] = point.accelerations[j] * scale * scale
            
           points[i] = point

       # Assign the modified points to the new trajectory
       new_traj.joint_trajectory.points = points

       # Return the new trajecotry
       return new_traj

## Function to plan and execute the spiral motion of the end-effector
def turn_thread(start, end, r, pivot, direction, undo):
  waypoints = []

  # start with the current pose
  waypoints.append(group.get_current_pose().pose)

  # first orient grippe
  wpose = geometry_msgs.msg.Pose()
  wpose.orientation.x = 0.0
  wpose.orientation.y = math.sqrt(0.5)
  wpose.orientation.z = 0.0
  wpose.orientation.w = math.sqrt(0.5)
  wpose.position.x = waypoints[0].position.x
  wpose.position.y = waypoints[0].position.y
  wpose.position.z = waypoints[0].position.z + 0.00002
  waypoints.append(copy.deepcopy(wpose))

  # spiral movement
  for i in range(start, end, undo):
    # Draw a spiral
      t = math.radians(i)
      wpose.position.x += pivot*r*t*math.cos(t) #pivot1: pivot = 1; pivot2: pivot = -1 
      waypoints.append(copy.deepcopy(wpose))
      wpose.position.y -= pivot*direction*r*t*math.sin(t) #CW: direction = 1; CCW: direction = -1
      waypoints.append(copy.deepcopy(wpose))

  (plan1, fraction) = group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold

  group.execute(plan1)  

## Function to move the thread to the right position after each turn
def move_thread(u, s):
  waypoints = []

  # start with the current pose
  waypoints.append(group.get_current_pose().pose)

  # move the thread (x,y)
  wpose = geometry_msgs.msg.Pose()
  wpose.orientation.x = 0.0
  wpose.orientation.y = math.sqrt(0.5)
  wpose.orientation.z = 0.0
  wpose.orientation.w = math.sqrt(0.5)
  wpose.position.x = waypoints[0].position.x + u
  wpose.position.y = waypoints[0].position.y + s
  wpose.position.z = waypoints[0].position.z
  waypoints.append(copy.deepcopy(wpose))

  (plan1, fraction) = group.compute_cartesian_path(
  			 waypoints,   # waypoints to follow
  			 0.01,        # eef_step
  			 0.0)         # jump_threshold
  rospy.sleep(1)

  scaled_traj = scale_trajectory_speed(plan1, 0.15)
  group.execute(scaled_traj)

## Main function
def move_robot():

  # Store initial values Mxi, Myi
  rospy.sleep(1)
  force_vector = Force.mean()
  mxi = force_vector[0]
  myi = force_vector[1]

  n = 90
  no = 90
  s = 0.015
  r = 0.0001825
  turn=0 # number of consecutive turns around different pivot
  same=0 # number of consecutive turns around same pivot
  down=0.005 # Move down if movement is from Pivot 1 to Pivot 2
  up=-0.005 # Move up if movement is from Pivot 2 to Pivot 1
  direction=0 # directon=1: CW P1, direction=2: CCW P1, direction=3: CW P2, direction=4: CCW P2
  previous=0 # last turn was about the same pivot than previous one
  ensure=0 # ensure that thread is tied around the same pivot or not
  release=0 # if torque value increases too much, release=1 and thread will be released
  end=0 # if thread is totally untied end=1

  # Move +1 cm in y
  move_thread(0, s)

  rospy.sleep(2)

  # Read torque values Mx, My
  force_vector = Force.mean()
  mx = force_vector[0]
  my = force_vector[1]

  # Return to initial pose
  move_thread(0, -s)

  rospy.sleep(2)

  # if Myi - My greater than threshold value
  if (abs(myi - my)>0.085):
	# if torque Mx is smaller than initial torque Mxi, the movement is CW P1 until first turn complete
	if mx<mxi:
		turn_thread(n, n+270, r, 1, 1, 1)
		direction=1

	# else: turn CCW P2 until first turn complete
	else:
		turn_thread(n, n+270, r, -1, -1, 1)
		direction=4

  #else: move -1 cm in y
  else:
	move_thread(0, -s)

	rospy.sleep(2)

	# Read torque values Mx, My
	force_vector = Force.mean()
	mx = force_vector[0]
	my = force_vector[1]

	# Return to initial pose
	move_thread(0, s)

	rospy.sleep(2)

	# if torque is in Pivot 1 direction, then turn CCW P1 until turn complete
	if mx<mxi:
		turn_thread(n, n+270, r, 1, -1, 1)
		direction=2

	# else: turn CW P2 until turn complete
	else:
		turn_thread(n, n+270, r, -1, 1, 1)
		direction=3

  n=n+270
  rospy.sleep(4)

  turn=1

  # Continue untying the thread
  while True:

	# Read torque values Mx, My
	force_vector = Force.mean()
	mx = force_vector[0]
	my = force_vector[1]
	
	# if Myi - My greater than threshold value, continue turning the same direction about the same pivot
	if(abs(myi - my)>0.085):

		# if the gripper moved to ensure it was tied around the same pivot or not, undo the movement
		if(ensure==1):
			if(direction==1 or direction==4):
				move_thread(0, -0.003)
			elif (direction==2 or direction==3):
				move_thread(0, 0.003)
			
			rospy.sleep(2)

		# if last movement was around the same pivot as the previous one, then undo the movement to untighten the thread
		if(previous==1):
			if(direction==1): 
				move_thread(0, -(0.02/turn-0.001*(same-1)))
			elif(direction==2):
				move_thread(0, 0.02/turn-0.001*(same-1))
			elif(direction==3):
				move_thread(0, 0.02/turn-0.001*(same-1))
			else:
				move_thread(0, -(0.02/turn-0.001*(same-1)))

			rospy.sleep(2)	

		# if it is the first time the gripper turns about the same pivot, reduce the radius of movement
		else:
			r=r-(0.0001825-0.000135)/(turn+same)
		
		# if previous turn CW about P1, move to new x and y positions and keep turning CW P1
		if(direction==1):
			if(previous==0):
				if(abs(myi - my)>0.6):
					move_thread(0.0015, -0.03)
					release=1
				else:
					move_thread(0.001, -0.02)
					release=0
				rospy.sleep(2)

			turn_thread(n, n+360, r, 1, 1, 1)
			direction=1

		# if previous turn CCW about P1, keep turning CCW P1
		elif(direction==2):
			if(previous==0):
				if(abs(myi - my)>0.6):
					move_thread(0.0015, 0.03)
					release=1
				else:
					move_thread(0.001, 0.02)
					release=0
				rospy.sleep(2)

			turn_thread(n, n+360, r, 1, -1, 1)
			direction=2

		# if previous turn CW about P2, keep turning CW P2
		elif(direction==3):
			if(previous==0):
				if(abs(myi - my)>0.6):
					move_thread(-0.0015, 0.03)
					release=1
				else:
					move_thread(-0.001, 0.02)
					release=0
				rospy.sleep(2)
			
			turn_thread(n, n+360, r, -1, 1, 1)
			direction=3

		# if previous turn CCW about P2, keep turning CCW P2
		elif(direction==4):
			if(previous==0):
				if(abs(myi - my)>0.6):
					move_thread(-0.0015, -0.03)
					release=1					
				else:
					move_thread(-0.001, -0.02)
					release=0
				rospy.sleep(2)
			
			turn_thread(n, n+360, r, -1, -1, 1)
			direction=4

		rospy.sleep(6)

		force_vector = Force.mean()
		mx = force_vector[0]
		my = force_vector[1]
		
		if(abs(myi - my)<0.085):
			# Pull the thread to know if it is tied about the same pivot or not
			if(direction==1):
				move_thread(0, 0.02/turn-0.001*(same-1))
			elif (direction==2):
				move_thread(0, -(0.02/turn-0.001*(same-1)))
			elif(direction==3):
				move_thread(0, -(0.02/turn-0.001*(same-1)))
			else:
				move_thread(0, 0.02/turn-0.001*(same-1))
			previous=1	
		
		same=same+1
		n=n+360

		rospy.sleep(2)

	#else: find the good direction to turn about the other pivot
	else:
		# Move a little to make sure it is not tied about the same pivot
		if(direction==1):
			move_thread(0, 0.003)
		elif (direction==2):
			move_thread(0, -0.003)
		elif(direction==3):
			move_thread(0, -0.003)
		else:
			move_thread(0, 0.003)

		rospy.sleep(2)

		ensure=1

		# Read torque values Mx, My
		force_vector = Force.mean()
		mx = force_vector[0]
		my = force_vector[1]

		# if Myi - My smaller than threshold value, then find the direction to move about the othe pivot
		if(abs(myi - my)<0.085):

			# undo the previous movement
			if(direction==1):
				move_thread(0, -0.003)
			elif (direction==2):
				move_thread(0, 0.003)
			elif(direction==3):
				move_thread(0, 0.003)
			else:
				move_thread(0, -0.003)

			rospy.sleep(2)

			ensure=0

			# if last turn was about the same pivot than the previous one, then increase the radius of the movement
			if(previous==1):

				r=r+(0.0001825-0.000135)/(same+turn)

				# undo the previous movement to figure out which pivot to turn
				if(direction==1):
					move_thread(0, -(0.02/turn)-0.001*(same-1))
				elif(direction==2):
					move_thread(0, 0.02/turn-0.001*(same-1))
				elif(direction==3):
					move_thread(0, 0.02/turn-0.001*(same-1))
				else:
					move_thread(0, -(0.02/turn-0.001*(same-1)))

				rospy.sleep(2)

				# move the gripper to new x, y position
				if(direction==1):
					if(release==0):
						move_thread(-0.001, 0.02)
					else:
						move_thread(-0.0015, 0.03)

					rospy.sleep(2)

				elif(direction==2):
					if(release==0):
						move_thread(-0.001, -0.02)
					else:
						move_thread(-0.0015, -0.03)
					rospy.sleep(2)

				elif(direction==3):
					if(release==0):
						move_thread(0.001, -0.02)
					else:
						move_thread(0.0015, -0.03)
					rospy.sleep(2)

				elif(direction==4):
					if(release==0):
						move_thread(0.001, 0.02)
					else:
						move_thread(0.0015, 0.03)
					rospy.sleep(2)

			previous=0

			force_vector = Force.mean()
			mxi = force_vector[0]
			myi = force_vector[1]

			# if previous turn was about P1, move down. Else: move up
			if(direction==1 or direction==2):
				move_thread(down, 0)
			else:
				move_thread(up, 0)

			rospy.sleep(2)

			s = 0.015 + 0.001*(turn+same)

			# Move 1 cm + pivots' circumference*number of turns
			if (direction==1 or direction==4):
				move_thread(0, s)
			elif (direction==2 or direction==3):
				move_thread(0, -s)

			rospy.sleep(2)

			# Read torque values Mx, My
			force_vector = Force.mean()
			mx = force_vector[0]
			my = force_vector[1]

			# if Myi - My greater than threshold value, undo the movement
			if(abs(myi - my)>0.085):
				if (direction==1 or direction==4):
					move_thread(0, -s)
				elif (direction==2 or direction==3):
					move_thread(0, s)

			# else: undo half of the movement
			else:
				if (direction==1 or direction==4):
					move_thread(0, -s/2)
				elif (direction==2 or direction==3):
					move_thread(0, s/2)

			rospy.sleep(2)

			# if previous turn CW about P1
			if (direction==1):			

				# if Myi - My greater than threshold value, turn CCW about P2
	  			if(abs(myi - my)>0.085):
					direction=4
					turn_thread(n, n+360, r, -1, -1, 1)
					n=n+360
					rospy.sleep(5)
					
				# else: turn CW about P2
				else:
					direction=3
					no = n+180
					n = no
					turn_thread(n, n+170, r, -1, 1, 1)
					n=n+180
					rospy.sleep(3)

				turn = turn+1

			# if previous turn CCW about P1
			elif (direction==2):

				# if Myi - My greater than threshold value, turn CW about P2
	  			if(abs(myi - my)>0.085):
					direction=3
					turn_thread(n, n+360, r, -1, 1, 1)
					n=n+360
					rospy.sleep(5)
					
				# else: turn CCW about P2
				else:
					direction=4
					no = n+180
					n = no
					turn_thread(n, n+170, r, -1, -1, 1)
					n=n+180
					rospy.sleep(3)

				turn = turn+1

			# if previous turn CW about P2
			elif (direction==3):

				# if Myi - My greater than threshold value and number of turns is smaller than 3, turn CCW about P1
	  			if(abs(myi - my)>0.085 and n<=1080):
					direction=2
					turn_thread(n, n+360, r, 1, -1, 1)
					rospy.sleep(4)
					n=n+360

				# if Myi - My greater than threshold value and more than 3 turns, turn CCW about P1 and check if the force increases to finish the movement
				elif(abs(myi - my)>0.085 and n>1080):
					direction=2
					turn_thread(n, n+18, r, 1, -1, 1)
					rospy.sleep(2)
					force_vector = Force.mean()
					mx = force_vector[0]
					my = force_vector[1]

					#if torque has increased during the mvt, FINISH
					if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085)):
						end=1
						break
					else:
						turn_thread(n+18, n+25, r, 1, -1, 1)
						rospy.sleep(2)
						force_vector = Force.mean()
						mx = force_vector[0]
						my = force_vector[1]

						if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085)):
							end=1
							break
						else:
							turn_thread(n+25, n+45, r, 1, -1, 1)
							rospy.sleep(2)
							force_vector = Force.mean()
							mx = force_vector[0]
							my = force_vector[1]
							
							if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085)):
								end=1
								break
							else:
								turn_thread(n+45, n+90, r, 1, -1, 1)
								rospy.sleep(3)
								force_vector = Force.mean()
								mx = force_vector[0]
								my = force_vector[1]
								
								if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085)):
									end=1
									break
								else:
									turn_thread(n+90, n+325, r, 1, -1, 1)
									rospy.sleep(4)
									force_vector = Force.mean()
									mx = force_vector[0]
									my = force_vector[1]
									
									if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085)):
										end=1
										break
									else:
										turn_thread(n+325, n+335, r, 1, -1, 1)
										rospy.sleep(2)
										force_vector = Force.mean()
										mx = force_vector[0]
										my = force_vector[1]
										
										if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085) and (n>1080)):
											end=1
											break
										else:
											turn_thread(n+335, n+360, r, 1, -1, 1)
											n=n+360
																			
				
				# else: turn CW about P1
				elif(abs(myi - my)<0.085 and n<=1080):
					direction=1
					no = n+180
					n = no
					turn_thread(n, n+170, r, 1, 1, 1)
					rospy.sleep(4)
					n=n+180
					
				elif(abs(myi - my)<0.085 and n>1080):
					direction=1
					no = n+180
					n = no
					turn_thread(n, n+120, r, 1, 1, 1)
					rospy.sleep(4)
					force_vector = Force.mean()
					mx = force_vector[0]
					my = force_vector[1]

					#if torque has increased during the mvt, FINISH
					if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085)):
						end=1
						break
					else:
						turn_thread(n+120, n+130, r, 1, 1, 1)
						rospy.sleep(2)
						force_vector = Force.mean()
						mx = force_vector[0]
						my = force_vector[1]
						
						if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085)):
							end=1
							break
						else:
							turn_thread(n+130, n+138, r, 1, 1, 1)
							rospy.sleep(2)
							force_vector = Force.mean()
							mx = force_vector[0]
							my = force_vector[1]
							
							if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085)):
								end=1
								break
							else:
								turn_thread(n+138, n+145, r, 1, 1, 1)
								rospy.sleep(2)
								force_vector = Force.mean()
								mx = force_vector[0]
								my = force_vector[1]
								
								if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085)):
									end=1
									break
								else:
									turn_thread(n+145, n+160, r, 1, 1, 1)
									rospy.sleep(2)
									force_vector = Force.mean()
									mx = force_vector[0]
									my = force_vector[1]
									
									if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085)):
										end=1
										break
									else:
										turn_thread(n+160, n+170, r, 1, 1, 1)
										n=n+180

				rospy.sleep(3)
				turn = turn+1
				
			#if previous turn CCW about P2
			elif (direction==4):

				# if Myi - My greater than threshold value and less than 3 turns, turn CW about P1
	  			if(abs(myi - my)>0.085 and n<=1080):
					direction=1
					turn_thread(n, n+360, r, 1, 1, 1)
					rospy.sleep(4)
					n=n+360
				
				# if Myi - My greater than threshold value and more than 3 turns, turn CW about P1 and check if the thread has been totally untied	
				elif(abs(myi - my)>0.085 and n>1080):
					direction=1
					turn_thread(n, n+18, r, 1, 1, 1)
					rospy.sleep(2)
					force_vector = Force.mean()
					mx = force_vector[0]
					my = force_vector[1]
				
					#if torque has increased during the mvt, FINISH
					if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085)):
						end=1
						break
					else:
						turn_thread(n+18, n+25, r, 1, 1, 1)
						rospy.sleep(2)
						force_vector = Force.mean()
						mx = force_vector[0]
						my = force_vector[1]
						
						if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085)):
							end=1
							break
						else:
							turn_thread(n+25, n+45, r, 1, 1, 1)
							rospy.sleep(2)
							force_vector = Force.mean()
							mx = force_vector[0]
							my = force_vector[1]
							
							if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085)):
								end=1
								break
							else:
								turn_thread(n+45, n+90, r, 1, 1, 1)
								rospy.sleep(3)
								force_vector = Force.mean()
								mx = force_vector[0]
								my = force_vector[1]
								
								if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085)):
									end=1
									break
								else:
									turn_thread(n+90, n+325, r, 1, 1, 1)
									rospy.sleep(4)
									force_vector = Force.mean()
									mx = force_vector[0]
									my = force_vector[1]
									
									if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085)):
										end=1
										break
									else:
										turn_thread(n+325, n+335, r, 1, 1, 1)
										rospy.sleep(2)
										force_vector = Force.mean()
										mx = force_vector[0]
										my = force_vector[1]
										
										if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085)):
											end=1
											break
										else:
											turn_thread(n+335, n+360, r, 1, 1, 1)
											n=n+360
															
				# else: turn CCW about P1
				elif(abs(myi - my)<0.085 and n<=1080):
					direction=2
					no = n+180
					n = no
					turn_thread(n, n+170, r, 1, -1, 1)
					rospy.sleep(4)
					n=n+180

				elif(abs(myi - my)<0.085 and n>1080):
					direction=2
					no = n+180
					n = no
					turn_thread(n, n+120, r, 1, -1, 1)
					rospy.sleep(4)
					force_vector = Force.mean()
					mx = force_vector[0]
					my = force_vector[1]
					
					#if torque has increased during the mvt, FINISH
					if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085)):
						end=1
						break
					else:
						turn_thread(n+120, n+130, r, 1, -1, 1)
						rospy.sleep(2)
						force_vector = Force.mean()
						mx = force_vector[0]
						my = force_vector[1]
						
						if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085)):
							end=1
							break
						else:
							turn_thread(n+130, n+138, r, 1, -1, 1)
							rospy.sleep(2)
							force_vector = Force.mean()
							mx = force_vector[0]
							my = force_vector[1]
							
							if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085)):
								end=1
								break
							else:
								turn_thread(n+138, n+145, r, 1, -1, 1)
								rospy.sleep(2)
								force_vector = Force.mean()
								mx = force_vector[0]
								my = force_vector[1]
								
								if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085)):
									end=1
									break
								else:
									turn_thread(n+145, n+160, r, 1, -1, 1)
									rospy.sleep(2)
									force_vector = Force.mean()
									mx = force_vector[0]
									my = force_vector[1]
									
									if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085)):
										end=1
										break
									else:
										turn_thread(n+160, n+170, r, 1, -1, 1)
										n=n+180

				rospy.sleep(3)
				turn = turn+1
				
				if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085) and (n>1080) and (direction==1 or direction==2)):
					end=1
					break

			if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085) and (n>1080) and (direction==1 or direction==2)):
				end=1
				break

		if((abs(mxi - mx)>0.085 or abs(myi - my)>0.085) and (n>1080) and (direction==1 or direction==2)):
			end=1
			break

	if((end==1) and (n>1080) and (direction==1 or direction==2)):
		break			

if __name__=='__main__':
  try:
    move_robot()
  except rospy.ROSInterruptException:
    pass
moveit_commander.os._exit(0)
