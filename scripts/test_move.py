#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Time
from std_msgs.msg import Header
from std_msgs.msg import Duration
import numpy as np

import time

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint





def move_to_joint_angles(q1,q2,q3,q4,q5):
	msg = JointTrajectory()
	msg.joint_names = ["arm_base_joint","shoulder_joint", "bottom_wrist_joint", "elbow_joint","top_wrist_joint"]
	point = JointTrajectoryPoint()
	point.positions = [q1,q2,q3,q4,q5]
	point.time_from_start = rospy.Duration(1)
	msg.header.stamp = rospy.Time.now()
	msg.points.append(point)
	pub.publish(msg)
	rospy.sleep(8)
	

if __name__ == '__main__':
	
	pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
	rospy.init_node('joint_trajectory_publisher')
	rate = rospy.Rate(1) # 10hz
	move_to_joint_angles(0,0,0,0,0)
	move_to_joint_angles(0,0.8,-0.4,0,-0.8)    #marker 11
	move_to_joint_angles(0,1.5,-0.4,0,-0.8)    #marker 12
	move_to_joint_angles(-1.57,1.2,-0.4,0.2,-1)#marker 13	
	move_to_joint_angles(1.35,1.7,-0.4,0,-0.8) #marker 14
	move_to_joint_angles(3,0.3,-0.2,0,-0.3)    #marker 15
	move_to_joint_angles(3,0.3,0.4,0,-0.3)     #marker 16
	move_to_joint_angles(0.55,1.7,-0.4,0,-0.8) #marker 17

