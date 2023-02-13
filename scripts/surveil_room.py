#! /usr/bin/env python

"""

.. module:: surveil_room
  :platform: Unix
  :synopsis: Pyhton module for the SurveilRoom action server

.. moduleauthor:: Salvatore D'Ippolito

This module contains the node for the action server SurveilRoom, whose code is kept in the class MovingCamera.


"""

import random
import rospy
import time
import simple_colors

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Time
from std_msgs.msg import Header
from std_msgs.msg import Duration
import numpy as np

import time

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# Import the ActionServer implementation used.
from actionlib import SimpleActionServer
# Import custom message, actions and services.
from assignment2.msg import SurveilRoomFeedback, SurveilRoomResult
import assignment2  



class MovingCamera(object):

    """
    This class initiates the surveil_room Simple Action Server. This server simply moves the fourth joint in the arm so that the camera can rotate 
    and scan the room. 
    
    

    """

    def __init__(self):

        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer('surveil_room',
                                      assignment2.msg.SurveilRoomAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self._as.start()

        self.pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    
    
    def execute_callback(self, goal):

        success = True
        feedback = SurveilRoomFeedback()
        result = SurveilRoomResult()

        if goal is None :
            rospy.logerr('No surveil_room goal provided! This service will be aborted!')
            self._as.set_aborted()
            return
        
        joint_angle = -3.14

        while joint_angle < 3.14:
            self.move_to_joint_angles([-1.57, -1.57, 1.57, joint_angle, 0])
            joint_angle += 0.628
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                success = False
                return
        
        self.move_to_joint_angles([-1.57, -1.57, 1.57, 0, 0])
        
        if success:
            
            result.done = True
            self._as.set_succeeded(result)
            time.sleep(0.5)
            print('\n Room surveiled \n')
            return


    def move_to_joint_angles(self,c):
        #print("1")
        msg = JointTrajectory()
        #print("2")
        msg.joint_names = ["arm_base_joint","shoulder_joint", "bottom_wrist_joint", "elbow_joint","top_wrist_joint"]
        point = JointTrajectoryPoint()
        #print("3")
        point.positions = [c[0],c[1],c[2],c[3],c[4]]
        #print(point.positions)
        point.time_from_start = rospy.Duration(1)
        msg.header.stamp = rospy.Time.now()
        msg.points.append(point)
        #print("4")
        self.pub.publish(msg)
        rospy.sleep(2)
        #print(msg)


if __name__ == '__main__':

    rospy.init_node('surveil_room_action_server')
    server = MovingCamera()
    rospy.spin()
