#!/usr/bin/env python

"""

.. module:: home_surveilance_state_machine
  :platform: Unix
  :synopsis: Pyhton module for the finite state machine node

.. moduleauthor:: Salvatore D'Ippolito

This module contains the code of this simulation's main node, the 'surveyor_state_machine' node. It instantiates the robot's finite state 
machine architecture and coordinates the robot's behaviour through the use of the AgentState class.
The finite state machine used is built using the smach_ros architecture so all states are defined as classes, in this version of the code 5 states have
been used to characterize the robot's behaviour, so five classes have been made.

"""


import roslib
import rospy
import smach
import smach_ros
import time
import random
import simple_colors
from ontology_interface import HandleOntology, ChoosingMove

from armor_api.armor_client import ArmorClient

from assignment2.msg import ExecuteMoveFeedback, ExecuteMoveResult, ExecuteMoveGoal, ExecuteMoveAction
#from assignment2.msg import ChooseMoveFeedback, ChooseMoveResult, ChooseMoveGoal, ChooseMoveAction
from assignment2.msg import SurveilRoomFeedback, SurveilRoomResult, SurveilRoomGoal, SurveilRoomAction

from agent_interface import AgentState

# Import ROS-based messages.
from std_msgs.msg import Bool



LINE=50

class Load_Map(smach.State):
    """
    This class handles the initial state of the robot's state machine, where the robot's floor ontology is built using the HandleOntology
    class defined in the ontology_interface module.

    """
    def __init__(self):

        smach.State.__init__(self, outcomes=['map_loaded'],
                                   output_keys=['rooms'])

        self.handle_ontology=HandleOntology()

    def execute(self, userdata):

        rospy.loginfo('Executing state LOAD MAP')
        
        
        userdata.rooms = self.handle_ontology.read_room_create_ontology()
        

        return 'map_loaded'

       
        

class Charging(smach.State):
    """
    This class simulates the agent's charging state, the finite state machine transitions to this state whenever the robot state node notifies 
    that the robot is low on battery. This state also calls the execute_move action server to move the robot towards its charging station room
    before initiating the charging sequence.

    This class publishes on the state/reached_charging_point topic to notify the robot_state node that the robot has reached the charging point
    and can start the charging sequence.

    """
    def __init__(self,agent_interface):
        self.comunicate_to_agent = agent_interface  
        smach.State.__init__(self, outcomes=['full_charge'])
        self._charging_point_reached=False

        # Set up as publisher on state/reached_charging_point
        self.publisher1 = rospy.Publisher('state/reached_charging_point', Bool, queue_size=1, latch=True)

    def execute(self, userdata):

        print('[{0}]\n'.format(simple_colors.cyan('='*LINE)))
        rospy.loginfo('Executing state CHARGING')
        
        self._charging_point_reached=False
        self.publisher1.publish(Bool(self._charging_point_reached))

        # Reaching the charging station:
        #-----------------------------------------------------------------
        goal_coordinates=[1.5, 8.0]
        goal = ExecuteMoveGoal(move_to='E', x=goal_coordinates[0], y=goal_coordinates[1])        
        self.comunicate_to_agent.execute_move_client.send_goal(goal)


        print('\n  Moving to charging station\n {}\n'.format('-'*int(LINE/4)))

        self.comunicate_to_agent.execute_move_client.wait_for_result()

        self._charging_point_reached=True
        self.publisher1.publish(Bool(self._charging_point_reached))
    

        # Waiting for battery to charge:
        #----------------------------------------------------------------
        print('\n  Charging Station reached, initiating charging sequence\n {}\n'.format('-'*int(LINE/4)))
        battery_level=0

        # wait until charged
        while  self.comunicate_to_agent.is_battery_low():

            pass

        print('\n   Done Charging\n')
             
        return 'full_charge'

class Choose_Move(smach.State):
    """
    This class instantiates the Choose Move state , in this state the robot chooses where to go based on what rooms are imediately reachable
    according to the ontology. The chosen destination gets passed on to the Execute Move state through the state machine's userdata structure. 
    
    It's a very short state that has only one outcome that triggers the Execute Move state.

    """
    def __init__(self):
        
        self.choose_move = ChoosingMove()
        
        smach.State.__init__(self,outcomes=['move_chosen'],
                                  output_keys=['chosen_destination'])

    def execute(self,userdata):

        print('[{0}]\n'.format(simple_colors.cyan('='*LINE)))
        rospy.loginfo('Executing state CHOOSE_MOVE')
        
        userdata.chosen_destination = self.choose_move.chose_room()
        
        return 'move_chosen'


class Execute_Move(smach.State):
    """
    This class launches the execute_move action server, in this state the robot moves to the location chosen in the Choose Move state, that has been passed through
    the userdata structure. The action server request can get preempted if the battery state changes to low during the server's execution.

    """
    def __init__(self,agent_interface):

        self.comunicate_to_agent = agent_interface
        smach.State.__init__(self, outcomes=['move_completed','go_charge'],
                                   input_keys=['chosen_destination','rooms'])

    def execute(self, userdata):

        print('[{0}]\n'.format(simple_colors.cyan('='*LINE)))
        rospy.loginfo('Executing state EXECUTE_MOVE')       
        
        goal_coordinates=userdata.rooms[userdata.chosen_destination]
        print("going to room "+userdata.chosen_destination+" at coordinates: "+str(goal_coordinates))
        goal = ExecuteMoveGoal(move_to=userdata.chosen_destination, x=goal_coordinates[0], y=goal_coordinates[1])        
        self.comunicate_to_agent.execute_move_client.send_goal(goal)

        while not rospy.is_shutdown(): 

            self.comunicate_to_agent.mutex.acquire()
            try:
                               
                if  self.comunicate_to_agent.is_battery_low():
                    print('\n')
                    rospy.loginfo(simple_colors.magenta('Interrupting state EXECUTE_MOVE since the battery is too low\n'))
                    self.comunicate_to_agent.execute_move_client.cancel_goals()  
                    return 'go_charge'

                if  self.comunicate_to_agent.execute_move_client.is_done():
                    return 'move_completed'

            finally:

                self.comunicate_to_agent.mutex.release()

class Survey_Room(smach.State):
    """
    This class describes the robot behaviour in the Survey Room state. In this version of the code a simple progression bar has been implemented to give an idea of the
    surveying room action's progress, in future versions another action server will probably be used as it has been done in the Choose Move and Execute Move states.
    
    """
    def __init__(self,agent_interface):
        self.comunicate_to_agent = agent_interface
        smach.State.__init__(self, outcomes= ['room_surveyed','go_charge'])

    def execute(self,userdata):

        print('[{0}]\n'.format(simple_colors.cyan('='*LINE)))
        rospy.loginfo('Executing state SURVEY_ROOM')

        self.delay=0.1
        self.survey_time=5
        time_passed=0
        print('\n')
        goal = SurveilRoomGoal(start = True)
        self.comunicate_to_agent.surveil_room_client.send_goal(goal)

        while not rospy.is_shutdown() :

            print('\r Surveyling room: [{0}{1}]'.format(simple_colors.blue('#'*int(20*time_passed)),'_'*int(20*(self.survey_time-time_passed))), end='')

            if  self.comunicate_to_agent.is_battery_low():
                print('\n')
                rospy.loginfo(simple_colors.magenta('Interrupting state SURVEY_ROOM since the battery is too low\n') )
                return 'go_charge'

            if self.comunicate_to_agent.surveil_room_client.is_done():
                
                return 'room_surveyed'
           

            time.sleep(self.delay)
            time_passed=(time_passed+self.delay)%self.survey_time
            





def main():

    rospy.init_node('surveyor_state_machine')
    
    agent_interface = AgentState()

    sm = smach.StateMachine(outcomes=['container_interface'])
    print('[{}]'.format(simple_colors.cyan('='*LINE)))
    print('{0} ROBOT SURVEILANCE FINITE STATE MACHINE {1}'.format(' '*int((LINE-40)/2),' '*int((LINE-40)/2)))
    print('[{}]'.format(simple_colors.cyan('='*LINE)))


    with sm:
       
        smach.StateMachine.add('LOAD_MAP', Load_Map(), 
                               transitions={'map_loaded':'CHOOSE_MOVE'})
                                            

        smach.StateMachine.add('CHARGING', Charging(agent_interface), 
                               transitions={'full_charge':'CHOOSE_MOVE'})

        smach.StateMachine.add('EXECUTE_MOVE', Execute_Move(agent_interface), 
                               transitions={'go_charge':'CHARGING',
                                            'move_completed':'SURVEY_ROOM'})

        smach.StateMachine.add('CHOOSE_MOVE', Choose_Move(), 
                               transitions={ 'move_chosen':'EXECUTE_MOVE'})

        smach.StateMachine.add('SURVEY_ROOM', Survey_Room(agent_interface), 
                               transitions={'go_charge':'CHARGING',
                                            'room_surveyed':'CHOOSE_MOVE'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    

    main()



