#!/usr/bin/env python

"""

.. module:: agent_interface
  :platform: Unix
  :synopsis: Python module that contains the classes ActionclientHelper and AgentState

.. moduleauthor:: Salvatore D'Ippolito

This module contains the classes called to interface the finite state machine node with the action clients and topics related 
to the robot agent moving in the armor ontology. 

The class ActionClientHelper has been taken as is from this repository: https://github.com/buoncubi/arch_skeleton                                                                   

"""


# Import ROS libraries.
import rospy

from actionlib import SimpleActionClient

# Import mutex to manage synchronization among ROS-based threads (i.e., node loop and subscribers)
from threading import Lock


# Import ROS-based messages.
from std_msgs.msg import Bool

from assignment2.msg import ExecuteMoveFeedback, ExecuteMoveResult, ExecuteMoveGoal, ExecuteMoveAction
from assignment2.msg import ChooseMoveFeedback, ChooseMoveResult, ChooseMoveGoal, ChooseMoveAction



# A class to simplify the implementation of a client for ROS action servers. It is used by the `InterfaceHelper` class.
class ActionClientHelper:

    """
    Class constructor, i.e., class initializer. Input parameters are:

    - `service_name`: it is the name of the server that will be invoked by this client.

    - `action_type`: it is the message type that the server will exchange

    - `done_callback`: it is the name of the function called when the action server completed its computation. If
      this parameter is not set (i.e., set to `None`), then only the `self._done_callback` function will be
      called when the server completes its computation.

    - `feedback_callback`: it is the name of the function called when the action server sends a feedback message. If
      this parameter is not set (i.e., set to `None`), then only the `self._feedback_callback` functions will be
      called when the server sends a feedback message.

    - `mutex`: it is a `Lock` object synchronised with the `done_callback` and `feedback_callback`. If it is not set
      (i.e., set to `None`), then a new mutex instance is considered. Set this variable if you want to extends the
      synchronization with other classes.
    

    """

    def __init__(self, service_name, action_type, done_callback=None, feedback_callback=None, mutex=None):
        # Initialise the state of this client, i.e.,  `_is_running`, `_is_done`, and `_results`.
        self.reset_client_states()
        # Set the name of the server to be invoked.
        self._service_name = service_name
        # Get or create a new mutex.
        if mutex is None:
            self._mutex = Lock()
        else:
            self._mutex = mutex
        # Instantiate a simple ROS-based action client.
        self._client = SimpleActionClient(service_name, action_type)
        # Set the done and feedback callbacks defined by the class using this client.
        self._external_done_cb = done_callback
        self._external_feedback_cb = feedback_callback
        # Wait for the action server to be alive.
        self._client.wait_for_server()

    # Start the action server with a new `goal`. Note this call is not blocking (i.e., asynchronous performed).
    def send_goal(self, goal):

        if not self._is_running:
            # Start the action server.
            self._client.send_goal(goal,
                                   done_cb=self._done_callback,
                                   feedback_cb=self._feedback_callback)
            # Set the client's states.
            self._is_running = True
            self._is_done = False
            self._results = None
        else:
            warn_msg = 'Warning send a new goal, cancel the current request first!'
         

    # Stop the computation of the action server.
    def cancel_goals(self):
        # The computation can be stopped only if the server is actually computing.
        if self._is_running:
            # Stop the computation.
            self._client.cancel_all_goals()
            # Reset the client's state.
            self.reset_client_states()
        else:
            warn_msg = 'Warning cannot cancel a not running service!'
        

    # Reset the client state variables stored in this class.
    def reset_client_states(self):
        self._is_running = False
        self._is_done = False
        self._results = None

    # This function is called when the action server sends some `feedback` back to the client.
    def _feedback_callback(self, feedback):

        # Acquire the mutex to synchronise the computation concerning the `feedback` message with the other nodes of the architecture.
        self._mutex.acquire()
        
        try:
            # Eventually, call the method provided by the node that uses this action client to manage a feedback.
            if self._external_feedback_cb is not None:
                self._external_feedback_cb(feedback)
            # Uncomment below to log information.
            # rospy.loginfo(anm.tag_log(f'`{self._service_name}` action server provide feedback: {feedback}.', LOG_TAG))
        finally:
            # Realise the mutex to (eventually) unblock ROS-based thread waiting on the same mutex.
            self._mutex.release()

    # This function is called when the action server finish its computation, i.e., it provides a `done` message.
    def _done_callback(self, status, results):
        # Acquire the mutex to synchronise the computation concerning the `done` message with the other nodes of the architecture.
        self._mutex.acquire()
        try:
            # Set the client's state
            self._is_running = False
            self._is_done = True
            self._results = results
            # Eventually, call the method provided by the node that uses this action client to manage a result.
            if self._external_done_cb is not None:
                self._external_done_cb(status, results)
            # Uncomment below to log information.
            # log_msg = f'`{self._service_name}` done with state `{self._client.get_state_txt()}` and result: {results}.'
            # rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        finally:
            self._mutex.release()

    # Get `True` if the action server finished is computation, or `False` otherwise.
    # Note that use this method should do it in a `self._mutex` safe manner.
    def is_done(self):  # they should be mutex safe
        return self._is_done

    # Get `True` if the action server is running, or `False` otherwise.
    # A note that use this method should do it in a `self._mutex` safe manner.
    def is_running(self):
        return self._is_running

    # Get the results of the action server, if any, or `None`.
    def get_results(self):
        if self._is_done:
            return self._results
        else:
            log_err = f'Error: cannot get result for `{self._service_name}`.'

            return None


class AgentState:

    """
    This class when instantiated, sets up two clients for the action servers ChooseMove and ExecuteMove.
    It also sets up as a subscriber to the battery state publisher node. It makes it possible for the 
    state machine node to retrieve information from, and give comands to, the  robot.

    """
    def __init__(self):

        self.mutex = Lock()
    
        self._battery_low = False
        self.remaining_meters=42

        rospy.Subscriber('state/battery_low', Bool, self.battery_callback)
       
        self.choose_move_client = ActionClientHelper('choose_move',ChooseMoveAction, mutex=self.mutex)

        self.execute_move_client = ActionClientHelper('execute_move',ExecuteMoveAction,feedback_callback = self.move_feedback_cb, mutex=self.mutex)




    
    def move_feedback_cb(self,feedback):

        """
        This is a feedback callback function for the ExecuteMove action server, it saves in a class variable the distance left 
        to reach the target passed to the server.

        """

        self.remaining_meters= feedback.meters_to_destination
        


    
    def meters_to_destination(self):

        """
        This method is called to retrieve the feedback value from the ExecuteMove action server

        """

        if self.remaining_meters>0:    
            
            return self.remaining_meters


  
    def battery_callback(self, msg):

        """
        This is a subscriber callback function from the topic 'state/battery_low', when the battery state gets updated, this callback saves it in a class variable

        """
        # Acquire the mutex to assure the synchronization with other subscribers and action clients (this assures data consistency).
        self.mutex.acquire()
        try:
            # Get the battery state and set it as the relative state variable encoded in this class.
            self._battery_low = msg.data

        finally:
            # Release the mutex to eventually unblock the other subscribers or action servers that are waiting.
            self.mutex.release()
    

    def is_battery_low(self):

        """
        This method gets called to check the robot's battery state, it returns the value previously set by the battery_callback function

        """
        return self._battery_low



