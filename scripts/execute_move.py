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
from assignment2.msg import ExecuteMoveFeedback, ExecuteMoveResult
import assignment2  


from armor_api.armor_client import ArmorClient
from armor_api.armor_manipulation_client import ArmorManipulationClient
from armor_api.armor_query_client import ArmorQueryClient
from armor_api.armor_utils_client import ArmorUtilsClient

import actionlib
from actionlib import SimpleActionClient, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped





from agent_interface import AgentState



class Moving2Location:

    """

    This class instantiates the execute_move action server. This server is tasked to move the robot both inside its ontology, and in the simulated 
    environment. It receives the desired new location as a request string and returns a boolean value as a response after the actual motion of the robot 
    has been completed. This server is itself a client to the move base action server.
    
    """

    def __init__(self):

        self.goal_x = 0.0

        self.goal_y = 0.0

        # Instantiating and starting the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer('execute_move',
                                      assignment2.msg.ExecuteMoveAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self._as.start()

        self.client = ArmorClient('surveyor_','map_ontology_')

        self.feedback = ExecuteMoveFeedback()
        self.result = ExecuteMoveResult()

        self.comunicate_to_agent =  AgentState()


    def execute_callback(self, goal):

        success = True

        if goal is None :
            rospy.logerr('No execute_move goal provided! This service will be aborted!')
            self._as.set_aborted()
            return
       
        print("new execute move going to: %.2f, %.2f" %(goal.x, goal.y))

        self.goal_x=goal.x
        self.goal_y=goal.y

        # Send goal to move_base
        self.try_goal( goal.x, goal.y)

       
        while not rospy.is_shutdown(): 

            self.comunicate_to_agent.mutex.acquire()
            try:
               
                # Cancel move_base goal if execute_move gets preempted               
                if  self._as.is_preempt_requested():
                    self.comunicate_to_agent.move_base_client.cancel_goals()
                    self._as.set_preempted()
                    success = False
                    print("execute_move preempted")
                    self.comunicate_to_agent.move_base_client.wait_for_result()

                    break

                if  self.comunicate_to_agent.move_base_client.is_done():
                    print("move base goal reached")
                    break

            finally:

                self.comunicate_to_agent.mutex.release()

        if success:
           
            self.move_robot(goal.move_to)
            
            self.result.location_reached = True 
            
            
            self._as.set_succeeded(self.result)
            
            return

    def try_goal(self,goal_x,goal_y):

        """

        This method prepares the goal message for the move_base server and sends it.

        """

        mb_goal=MoveBaseGoal()

        mb_goal.target_pose.header.frame_id = 'map'
        mb_goal.target_pose.header.stamp = rospy.Time.now()
        mb_goal.target_pose.pose.position.x = goal_x
        mb_goal.target_pose.pose.position.y = goal_y   
        mb_goal.target_pose.pose.orientation.w = 1.0

        # self.move_client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        self.comunicate_to_agent.move_base_client.send_goal(mb_goal)
        

    def move_robot(self,new_location):

        """
        This method moves the robot in its ontology. According to the rules of this ontology, this operation is comprised of three steps:
        - replacing the old value for the agent's object property 'isIn' with the new desired vaue.

        - updating the new locations's 'visitedAt' data property with the current timestamp.

        - updating the robot's own 'now' data property so that the ontology can reason on the urgency of new rooms.

        """
        
        # Finding the old location for the replace command
        old_location = self.client.query.objectprop_b2_ind('isIn','Robot1')
        old_location = old_location[0]
        old_location = old_location[32:-1]

        # Finding the visitedAt data property of the new location to replace it later 
        visited_at = self.client.query.dataprop_b2_ind('visitedAt', new_location)
        visited_at = visited_at[0]
        visited_at = visited_at[1:-11]
      
        now = self.client.query.dataprop_b2_ind('now', 'Robot1')
        now=now[0]
        now=now[1:-11]
        
        self.client.manipulation.replace_objectprop_b2_ind('isIn','Robot1',new_location,old_location)
        self.client.manipulation.replace_dataprop_b2_ind('visitedAt',new_location,'Long',str(int(time.time())),visited_at)
        self.client.manipulation.replace_dataprop_b2_ind('now','Robot1','Long',str(int(time.time())),now)
        
       
        print(simple_colors.green('\n Surveyor left location %s and reached %s \n' %(old_location,new_location)))

        # Launching reasoner after manipulations
        self.client.utils.sync_buffered_reasoner()
        

if __name__ == '__main__':

    rospy.init_node('execute_move_action_server')
    server = Moving2Location()
    rospy.spin()
