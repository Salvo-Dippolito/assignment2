#! /usr/bin/env python

"""

.. module:: choose_move
  :platform: Unix
  :synopsis: Pyhton module for the ChooseMove action server

.. moduleauthor:: Salvatore D'Ippolito

This module contains the node for the action server ChooseMove, whose code is kept in the class ChoosingMove.


"""

import random
import rospy
import time
import simple_colors

# Import the ActionServer implementation used.
from actionlib import SimpleActionServer
# Import custom message, actions and services.
from assignment2.msg import ChooseMoveFeedback, ChooseMoveResult
import assignment2  


from armor_api.armor_client import ArmorClient
from armor_api.armor_manipulation_client import ArmorManipulationClient
from armor_api.armor_query_client import ArmorQueryClient
from armor_api.armor_utils_client import ArmorUtilsClient



class ChoosingMove(object):

    """
    This class initiates the choose_move Simple Action Server. 
    
    

    """

    def __init__(self):

        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer('choose_move',
                                      assignment2.msg.ChooseMoveAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self._as.start()

        self.client = ArmorClient('surveyor_','map_ontology_')
    
    def clean_room_strings(self,room_list):

        clean_room_strings=[]
        for room_string in room_list:
            room_string=room_string[32:-1]
            clean_room_strings.append(room_string)
        return clean_room_strings  

    def classify_rooms(self,room_list):
        corridors=[]
        urgent_rooms=[]
        non_urgent_rooms=[]

        for room in room_list:

            if self.client.query.check_if_ind_b2_class(room,'CORRIDOR'):
                corridors.append(room)
            elif self.client.query.check_if_ind_b2_class(room,'URGENT'):
                urgent_rooms.append(room)
            else:
                non_urgent_rooms.append(room)

        return urgent_rooms, non_urgent_rooms, corridors 

    def execute_callback(self, goal):

        success = True
        feedback = ChooseMoveFeedback()
        result = ChooseMoveResult()

        if goal is None :
            rospy.logerr('No choose_move goal provided! This service will be aborted!')
            self._as.set_aborted()
            return
        
        if self._as.is_preempt_requested():
                self._as.set_preempted()
                success = False
                return


        self.client.utils.apply_buffered_changes()
        self.client.utils.sync_buffered_reasoner()
        reachable_rooms = self.client.query.objectprop_b2_ind('canReach','Robot1')
        print('\n Reachable rooms are: ')
        print(self.clean_room_strings(reachable_rooms))
        
        urgent_rooms, non_urgent_rooms, corridors=self.classify_rooms(self.clean_room_strings(reachable_rooms))

        print('Urgent rooms: ')
        print(urgent_rooms)
        print('Non urgent rooms: ')
        print(non_urgent_rooms)
        print('Corridors: ')
        print(corridors)

        if not urgent_rooms:

            if not corridors or not non_urgent_rooms:
                go_to = random.choice(non_urgent_rooms+corridors)
                
            else:

                if (random.uniform(0, 100)>60):
                    go_to = random.choice(non_urgent_rooms)
                else:
                    go_to = random.choice(corridors)

        else:

            go_to = random.choice(urgent_rooms)

        print('\n Surveyor will move to location %s \n' %go_to)

        result.chosen_room = go_to
        
        if success:
            self._as.set_succeeded(result)
            return



if __name__ == '__main__':

    rospy.init_node('choose_move_action_server')
    server = ChoosingMove()
    rospy.spin()
