#! /usr/bin/env python

"""

.. module:: execute_move

  :platform: Unix
  :synopsis: Pyhton module for the ExecuteMove action server

.. moduleauthor:: Salvatore D'Ippolito

This module holds the code for the read_room Simple Action Server. The sever is instantiated by the class Moving2Location.

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

from test_move2 import ReadRoomCreateOntology



    

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
        
        pippo=ReadRoomCreateOntology()


        if success:
            self._as.set_succeeded(result)
            return


if __name__ == '__main__':

    rospy.init_node('read_room_action_server')
    server = ReadFromPoses()
    rospy.spin()
