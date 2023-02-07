#!/usr/bin/env python

"""

.. module:: battery_state
  :platform: Unix
  :synopsis: Python module that contains the class BatteryState

.. moduleauthor:: Salvatore D'Ippolito

This module can start different nodes that handle some aspects of the robot's internal state. In this version of the code only a battery state notifier
is set up. 

Publishes to: state/battery_low

"""

import threading
from random import uniform
import rospy
import time
import simple_colors

# Import the messages used by services and publishers.
from std_msgs.msg import Bool




class RobotState:

    """

    This class can run the code of various publishers that update the states of some internal robot functions.

    In this iteration of the code only the battery state publisher is implemented. It is initiated on a separate thread 
    and publishes on the topic 'state/battery_low' where it periodically updates the state of the robot's battery.



    """

    def __init__(self):
      
        rospy.init_node('robot_state')

    # Battery publisher variables
    #---------------------------------------------------------

        # Start from a charged battery:
        self._battery_low = False
        self.max_battery=100
        
        #starts from a full charge, to start from an empty charge you must also have to change the sign of
        #self.battery_change
        self.battery_level=self.max_battery
        


        # Set seconds it takes to increase the charge by 1%
        self.delay = 0.3
        self.battery_change=1

        # Set this variable to true to test architecture with a random battery state notifier
        self.random = False       

    #
    #---------------------------------------------------------


        th = threading.Thread(target=self.is_battery_low)
        th.start()
    
    def is_battery_low(self):
        
        """
        This method holds the code for the battery state publisher. It simulates a charging and discharging cycle that runs independently
        from the rest of the code and publishes on the topic /state/battery_low if the battery is charged or not.

        If self.random in the RobotState class initializer is set to True then it will not simulate a charging cycle and instead will 
        change the state of the battery at random time intervals

        """

        if not self.random:

            publisher = rospy.Publisher('state/battery_low', Bool, queue_size=1, latch=True)
         
            while not rospy.is_shutdown():

            
                if self.battery_level>=self.max_battery:
                   
                    # Change battery state to high:
                    self._battery_low = False
                    
                    # Publish current state of the battery:
                    publisher.publish(Bool(self._battery_low))

                    self.battery_change=-self.battery_change/2000

                elif self.battery_level<=0:
                  
                    # Change battery state to low:
                    self._battery_low= True

                    # Publish current state of the battery:
                    publisher.publish(Bool(self._battery_low))
                    self.battery_change=-self.battery_change*2
                     

                    # Give the robot some time to reach the charging station
                    time.sleep(8)
                                 
                     
                        
                
                time.sleep(self.delay)
                self.battery_level=self.battery_level + self.battery_change

                if self._battery_low:


                    print('\r Battery level: [{0}{1}] {2}%'.format(simple_colors.yellow('#'*int(self.battery_level)),'_'*int(100-self.battery_level), self.battery_level), end='')
                
        else:

            publisher = rospy.Publisher('state/battery_low', Bool, queue_size=1, latch=True)
         
            while not rospy.is_shutdown():

                self.delay=uniform(1,30)
                print(simple_colors.red('\n RANDOM ROBOT STATE: Will wait for %d seconds for the battery to change state'%self.delay))
                time.sleep(self.delay)
                self._battery_low= True
                # Publish current state of the battery:
                publisher.publish(Bool(self._battery_low))
                # Give the robot some time to reach the charging station
                time.sleep(9)

                self.delay=uniform(1,30)
                print(simple_colors.red('\n RANDOM ROBOT STATE: Will wait for %d seconds for the battery to change state'%self.delay))
                time.sleep(self.delay)
                self._battery_low = False

                # Publish current state of the battery:
                publisher.publish(Bool(self._battery_low))



if __name__ == "__main__":
  

    RobotState()
    rospy.spin()



