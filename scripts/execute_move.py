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
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID



    

class Moving2Location:

    """
    This class instantiates the execute_move action server. This server is tasked to move the robot inside its ontology, it receives the desired new 
    location as a request string and returns a boolean value as a response after the motion of the robot has been completed.

    In a more complex scenario this server would also be tasked to actually control the robot's motion in a simulated environment, and not only in 
    its ontological representation. For now it simulates the time it might need the robot to move for some distance. This distance is currently set
    randomly by this same server. The progress done on this artificial distance is published as the server's feedback message.
    

    """

    def __init__(self):

        # Instantiating and starting the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer('execute_move',
                                      assignment2.msg.ExecuteMoveAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self._as.start()

        self.client = ArmorClient('surveyor_','map_ontology_')

        #Setting up as a client to the move_base server:
        self.move_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.move_client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
  
    def done_cb(self, status, result):

    # This callback interprets the received goal status for the user 
    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            print("  Received a cancel request\n")
            
        if status == 3:
            print("  Goal reached!\n ")
            

        if status == 4:
            print("  Can't reach desired target\n")
                     

        if status == 5:
            print("  Goal rejected by action server\n")
                    

    def active_cb(self):
        print("  we gon get there man '\n\r", flush=True)

    def feedback_cb(self, feedback ):
        
        #Uncomment this to have the feedback printed on the terminal: 
        #print(str(feedback))
        pass



    def execute_callback(self, goal):

        success = True
        feedback = ExecuteMoveFeedback()
        result = ExecuteMoveResult()

        # dist_to_location= uniform(0.5,4) # [m]
        # time_interval=0.1                # [s]
        # speed=0.5                        # [m/s]
        
        if goal is None :
            rospy.logerr('No execute_move goal provided! This service will be aborted!')
            self._as.set_aborted()
            return
        # send request to movebase here:



        print(goal.x)
        print(goal.y)

        self.try_goal( goal.x, goal.y)

        #waiting for a result from the action server:
        self.move_client.wait_for_result()

        # meters_traveled=0
        # while meters_traveled<dist_to_location :

        #     if self._as.is_preempt_requested():
        #         self._as.set_preempted()
        #         success = False
        #         return

        #     #simulating movement in time:
        #     rospy.sleep(time_interval)
        #     meters_traveled=meters_traveled+speed*time_interval
            
        #     feedback.meters_to_destination = dist_to_location-meters_traveled
        #     self._as.publish_feedback(feedback)

        self.move_robot(goal.move_to)
        result.location_reached = True   

        if success:
            self._as.set_succeeded(result)
            return

    def try_goal(self,goal_x,goal_y):

        goal=MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y   
        goal.target_pose.pose.orientation.w = 1.0

        self.move_client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        

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
