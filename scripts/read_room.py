#! /usr/bin/env python

"""

.. module:: execute_move

  :platform: Unix
  :synopsis: Pyhton module for the ExecuteMove action server

.. moduleauthor:: Salvatore D'Ippolito

This module holds the code for the execute_move Simple Action Server. The sever is instantiated by the class Moving2Location.

"""
import rospy
from random import  uniform
import simple_colors
import time

# Import the ActionServer implementation used.
from actionlib import SimpleActionServer
# Import custom message, actions and services.
from assignment2.msg import ReadRoomFeedback, ReadRoomResult
import assignment2  


from armor_api.armor_client import ArmorClient
from armor_api.armor_manipulation_client import ArmorManipulationClient
from armor_api.armor_query_client import ArmorQueryClient
from armor_api.armor_utils_client import ArmorUtilsClient



    

class ReadFromPoses:

    """
    This class instantiates the read_room action server. 

    """

    def __init__(self):

        # Instantiating and starting the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer('read_room',
                                      assignment2.msg.ReadRoomAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self._as.start()

        self.client = ArmorClient('surveyor_','map_ontology_')
        


    def execute_callback(self, goal):

        success = True
        feedback = ReadRoomFeedback()
        result = ReadRoomResult()


        
        if goal is None :
            rospy.logerr('No read_room goal provided! This service will be aborted!')
            self._as.set_aborted()
            return
        


            if self._as.is_preempt_requested():
                self._as.set_preempted()
                success = False
                return


            
            feedback.meters_to_destination = dist_to_location-meters_traveled
            self._as.publish_feedback(feedback)

        self.move_robot(goal.move_to)
        result.location_reached = True   

        if success:
            self._as.set_succeeded(result)
            return


    

if __name__ == '__main__':

    rospy.init_node('read_room_action_server')
    server = ReadFromPoses()
    rospy.spin()
