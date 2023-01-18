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
	rospy.sleep(1)
	

if __name__ == '__main__':
	
	pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
	rospy.init_node('joint_trajectory_publisher')
	rate = rospy.Rate(1) # 10hz
	move_to_joint_angles(1.57,0,0,0,0)
	move_to_joint_angles(0,1.57,0,0,0)
	move_to_joint_angles(0,0,1.57,0,0)
	move_to_joint_angles(0,0,0,1.57,0)
	move_to_joint_angles(0,0,0,0,1.57)
	
