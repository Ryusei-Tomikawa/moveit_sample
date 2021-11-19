#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import math

import rospy
import actionlib
import moveit_commander

from geometry_msgs.msg import Quaternion, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

SLEEP_TIME = 1.5

# configuration for moveit
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

# group name
upper = moveit_commander.MoveGroupCommander("upper_body")
rarm_waist = moveit_commander.MoveGroupCommander("rarm_with_waist")
lifter = moveit_commander.MoveGroupCommander("lifter")

lifter.set_pose_reference_frame("base_link")
upper.set_pose_reference_frame("base_link")
rarm_waist.set_pose_reference_frame("base_link")

def lifter_down():

	## reset pose
	upper_goal = upper.get_current_joint_values()
	for i in range(len(upper_goal)):
		upper_goal[i] = 0
	upper_goal[6] = -3
	upper_goal[16] = -3
	upper.go(upper_goal, wait=True)

	lifter_goal = lifter.get_current_joint_values()
	print("lifter get current state :=", lifter_goal)
	for i in range(len(lifter_goal)):
		lifter_goal[i] = 0
	lifter_goal[0] = 1.570131778717041
	lifter_goal[1] = -1.570131778717041
	lifter_goal[2] = -1.5704814195632935
	lifter_goal[3] = 1.5704814195632935
	lifter.set_max_velocity_scaling_factor(1.0)
	lifter.go(lifter_goal, wait=True)

	print("robot group:", robot.get_group_names())
	print ("")
	# print("robot current state:", robot.get_current_state())
	# print ("")
	#print("robot joint name:", robot.get_joint_names("upper") )

	rospy.sleep(SLEEP_TIME)

def lifter_up():
	## reset pose
	upper_goal = upper.get_current_joint_values()
	for i in range(len(upper_goal)):
		upper_goal[i] = 0
	upper_goal[6] = -3
	upper_goal[16] = -3
	upper.go(upper_goal, wait=True)

	lifter_goal = lifter.get_current_joint_values()
	print("lifter get current state :=", lifter_goal)
	for i in range(len(lifter_goal)):
		lifter_goal[i] = 0
	lifter.set_max_velocity_scaling_factor(0.5)
	lifter.go(lifter_goal, wait=True)

	# print("robot group:", robot.get_group_names())
	# print ("")
	# print("robot current state:", robot.get_current_state())
	# print ("")
	# print("robot joint name:", robot.get_joint_names("lifter") )

	rospy.sleep(SLEEP_TIME)



def main():
	
	rospy.init_node("lifter")

	i = 0
	count = 0
	while True:
		lifter_down()
		lifter_up()
		count += 1
		print("count :=", count)
	
		


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		exit()
