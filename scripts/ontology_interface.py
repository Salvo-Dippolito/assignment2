#! /usr/bin/env python3

"""

.. module:: load_ontology
  :platform: Unix
  :synopsis: Python module that contains the class CreateFloorOntology

.. moduleauthor:: Salvatore D'Ippolito


This module contains the HandleOntology class, which is used to load and set up the floor ontology for the surveilance robot.


"""

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Time
from std_msgs.msg import Header
from std_msgs.msg import Duration
import numpy as np

import time

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from assignment2.srv import RoomInformation, RoomInformationRequest, RoomInformationResponse


# Import the armor client class
from armor_api.armor_client import ArmorClient
from os.path import dirname, realpath
from armor_msgs.srv import ArmorDirective, ArmorDirectiveList, ArmorDirectiveListRequest
from armor_msgs.msg import _ArmorDirectiveReq
from armor_api.armor_manipulation_client import ArmorManipulationClient
from armor_api.armor_query_client import ArmorQueryClient
from armor_api.armor_utils_client import ArmorUtilsClient


import random



class HandleOntology():
	"""
	This class loads the basic ontology created for this project, where no individuals apart from the robot agent have been defined, 
	but all class definitions and rules have already been set. It interacts with the user to define how many corridors constitute the floor that has to be surveiled
	and how many rooms are connected to each corridor. The user is also asked to set up the robot's 'urgencyThreshold' which s a data property that specifies 
	after how many seconds a room should be considered 'urgent' to visit. 
	"""

	def __init__(self):

		# publish on the arm topic to control it
		self.pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

		# set up a client to the ontology server
		self.client = ArmorClient('surveyor_','map_ontology_')
		self.locations_list=[]
		self.doors_list=[]
		
		# load basic ontology
		path = dirname(realpath(__file__)) + '/../ontologies/topological_map.owl'
		self.client.utils.load_ref_from_file(path, 'http://bnc/exp-rob-lab/2022-23', True, 'PELLET', True, False)
		self.client.utils.mount_on_ref()

		self.configurations=[ [0,0.8,-0.4,0,-0.8],[0,1.5,-0.4,0,-0.8],[-1.37,0.8,-0.4,0.5,-1],[1.35,1.7,-0.4,0,-0.8],[3.2,0.1,-0.2,0,-0.3],[3,0.3,0.4,0,-0.3],[0.55,1.7,-0.4,0,-0.8]]
		self.current_configuration=[0,0,0,0,0]


		# This is used for reference when printing on screen:
		self.LINE=50

	def read_room_create_ontology(self):

		"""
		This is the specific method that modifies the base ontology saved in the topological_map.owl file, which gets loaded by this class's initializer. 


		In this version of the code, the floors that can be created by this function all follow the same general scheme: all corridors are connected to a main corridor 
		denoted 'E' and all subsequent corridors are connected to each other. No limit has been put in place to control the number of rooms or corridor a user can add.
		
		"""

		print("\n============================================================\n")

		# Go to home position
		self.move_to_joint_angles([0,0,0,0,0])
		
		# set up room coordinates dictionary
		room_coordinates_dict={}

		# go through each pose and update the room coordinates dictionary
		for configuration in self.configurations:
			self.current_configuration=configuration
			self.move_to_joint_angles(configuration)

			# call server to process image and retrieve room info
			room,coordinates=self.get_room_layout_service_client()
			room_coordinates_dict[room]=coordinates

		print(room_coordinates_dict)

		self.old_value = self.client.query.dataprop_b2_ind('urgencyThreshold','Robot1')
		self.old_value = self.old_value[0]
		self.old_value = self.old_value[1:-11]

		self.move_to_joint_angles([0,0,0,0,0])
		self.move_to_joint_angles([-1.57, -1.57, 1.57, 0, 0])
		print('     {}'.format('-'*int(self.LINE/2)))
		print('\n Defining an urgency threshold for all rooms\n')
		print(' The default urgency threshold is: '+self.old_value+' seconds\n')
		self.urgency_threshold=self.get_number_of('seconds that pass before a room becomes urgent')
		self.client.manipulation.replace_dataprop_b2_ind( 'urgencyThreshold', 'Robot1', 'Long' , str(self.urgency_threshold), self.old_value)
		self.client.manipulation.add_objectprop_to_ind('isIn', 'Robot1', 'E')
		self.client.manipulation.disjoin_all_inds(self.doors_list)
		self.client.manipulation.disjoin_all_inds(self.locations_list)
		self.client.utils.apply_buffered_changes()
		self.client.utils.sync_buffered_reasoner()
		print(' The new urgency threshold is : %s'%(self.client.query.dataprop_b2_ind('urgencyThreshold','Robot1')))
		print('\n     {}\n\n'.format('-'*int(self.LINE/2)))

		return room_coordinates_dict
	

	def create_floor_ontology(self):
		"""
		This is the specific method that modifies the base ontology saved in the topological_map.owl file, which gets loaded by this class's initializer. 
		According to the user's specifications new individuals of class 'location' will be added to represent the floor's various rooms and corridors.
		Finally, it places the robot agent in its initial position and sets up all the needed data properties.

		In this version of the code, the floors that can be created by this function all follow the same general scheme: all corridors are connected to a main corridor 
		denoted 'E' and all subsequent corridors are connected to each other. No limit has been put in place to control the number of rooms or corridor a user can add.
		
		"""

		self.add_location('E')
		n=k=1
		print('\n')
		number_of_corridors=self.get_number_of('corridors')
		
		for i in range(1,number_of_corridors+1):

			corridor_name = 'C'+str(i)
			self.add_location(corridor_name)
			print('\n')
			rooms_per_corridor=self.get_number_of('rooms for this corridor')
			
			for j in range(1,rooms_per_corridor+1):
				
				room_name = 'R'+str(n)
				self.add_location(room_name)
				door_name = 'D'+str(n)
				self.connect_locations(room_name,corridor_name,door_name)
				n=n+1
				
			if i<number_of_corridors:

				#CONNECT CORRIDORS TOGETHER IF THERE ARE MORE THAN ONE 
				door_name = 'd'+str(k)
				k=k+1
				next_corridor_name='C'+str(i+1)
				self.connect_locations(corridor_name,next_corridor_name,door_name)

			door_name = 'd'+str(k)
			k=k+1
			self.connect_locations(corridor_name,'E',door_name)

		# INITIALIZING ROBOT PROPERTIES
		
		old_value = self.client.query.dataprop_b2_ind('urgencyThreshold','Robot1')
		old_value = old_value[0]
		old_value = old_value[1:-11]

		print('     {}'.format('-'*int(self.LINE/2)))
		print('\n Defining an urgency threshold for all rooms\n')
		print(' The default urgency threshold is: '+old_value+' seconds\n')		
		urgency_threshold=self.get_number_of('seconds that pass before a room becomes urgent')

		self.client.manipulation.replace_dataprop_b2_ind( 'urgencyThreshold', 'Robot1', 'Long' , str(urgency_threshold), old_value)
		self.client.manipulation.add_objectprop_to_ind('isIn', 'Robot1', 'E')
		self.client.manipulation.disjoin_all_inds(self.doors_list)
		self.client.manipulation.disjoin_all_inds(self.locations_list)

		
		self.client.utils.apply_buffered_changes()
		self.client.utils.sync_buffered_reasoner()
		print(' The new urgency threshold is : %s'%(self.client.query.dataprop_b2_ind('urgencyThreshold','Robot1')))

		print('\n     {}\n\n'.format('-'*int(self.LINE/2)))

	def add_location(self,name_of_location):
		"""
		This method is called when locations have to be added to the ontology, apart from adding a new individual of class 'LOCATION', it also sets the new location's 
		'visitedAt' data property which is needed for the reasoner to deduce which rooms should the robot prioritize. 

		"""

		#Initializing to a zero timestamp all new locations
		start_time=str(0)
		self.client.manipulation.add_ind_to_class(name_of_location, 'LOCATION')
		print('Added individual %s of type LOCATION' %name_of_location)
		self.locations_list.append(name_of_location)
		self.client.manipulation.add_dataprop_to_ind('visitedAt',name_of_location,'Long',start_time)

	def connect_locations(self,location1,location2,door_name):
		"""
		This method can be called to join two locations in the ontology by assigning to both locations the same 'door' individual, which gets created explicitly for this purpose.
		The ontology's reasoner will then deduce automatically that the two locations must be connected to each other.

		"""

		self.client.manipulation.add_ind_to_class(door_name, 'DOOR')
		self.doors_list.append(door_name)
		print('Added individual %s of type DOOR' %door_name)
		self.client.manipulation.add_objectprop_to_ind('hasDoor', location1, door_name)
		self.client.manipulation.add_objectprop_to_ind('hasDoor', location2, door_name)
		print('Connected %s and %s with door %s \n' %(location1, location2, door_name))
		

	def get_number_of(self,what):
		"""
		Simple method to get a valid number from the user
		
		"""

		x=''
		not_an_int = True
		while(not_an_int):
			x= input('Enter the number of %s: ' %what)
			try:
				int(x)
			except:
				print('\n Please enter a valid integer')
			else:
				not_an_int=False
		return int(x)

	def marker_id_service_client(self):
		"""
		This object sends a trigger request to the marker_service_id server. This server processes an incoming raw image message
		so that it can detect a valid marker id and return it as a response to the client. If the server couldn't send a valid 
		marker id, then this client moves the arm a bit and tries sending the request again.

		"""

		rospy.wait_for_service('/get_marker_id')
		print("marker_id_ok")
		try:
			service_handle = rospy.ServiceProxy('/get_marker_id', Trigger)
			req = TriggerRequest()
			res = service_handle(req)

			while not res.success:

				print("couldn't read marker, jiggling the arm a bit")
				self.jiggle( self.current_configuration)
				res = service_handle(req)
			
			print("read marker id: "+str(res.message))
			return res.message
		except rospy.ServiceException as e:

			print("Service call failed: %s" % e)

	def get_room_layout_service_client(self):
		"""
		This object calls the /room_info server to get all the room info linked to the marker that has been detected by the marker_service_id server.
		The room and all its connections are then added as individuals to the ontology file and room name and coordinates are returned.

		"""
		rospy.wait_for_service('/room_info')
		try:

			service_handle = rospy.ServiceProxy('/room_info', RoomInformation)

			req= RoomInformationRequest()

			# send the marker id found from the other server
			req.id=int(self.marker_id_service_client())
			res = service_handle(req)
			# add room to ontology

			print("adding location "+res.room)

			self.add_location(res.room)

			# add all the connections to the room
			for connection in res.connections:
				print("connecting location "+res.room+" with location "+connection.connected_to +" with door name "+connection.through_door)
				self.connect_locations(res.room,connection.connected_to, connection.through_door)
				print("\n============================================================\n")

				# return new room name and coordinates

			return res.room, [res.x,res.y]

		except rospy.ServiceException as e:

			print("Service call failed: %s" % e)

	def move_to_joint_angles(self,c):
		"""
		Object built to move the robot's arm to a desired configuration. 
		It publishes on the topic /arm_controller/command.

		"""

		msg = JointTrajectory()
		
		msg.joint_names = ["arm_base_joint","shoulder_joint", "bottom_wrist_joint", "elbow_joint","top_wrist_joint"]
		point = JointTrajectoryPoint()
		
		point.positions = [c[0],c[1],c[2],c[3],c[4]]
		
		point.time_from_start = rospy.Duration(1)
		msg.header.stamp = rospy.Time.now()
		msg.points.append(point)
		
		self.pub.publish(msg)
		rospy.sleep(2)
		
		

	def jiggle(self,old_configuration):
		"""
		Function to move the arm a bit around a given configuration. Used to slightly change the
		camera's orientation when it's not able to read an ARUCO marker.

		"""
		
		new_config= old_configuration+ np.random.uniform(-0.1, 0.1, 5)
		self.move_to_joint_angles( new_config)
		print ("moved to ")
		print(new_config)


class ChoosingMove(object):

    """
    This class was also used in the ChooseMove action server, it executes the server's code. 
   	It's placed in this node along with other classes that have to interact with the ontology.
    It's a client to the armor server and chooses in which room the robot should move to after asking for a list of all reachable rooms.
    
    """

    def __init__(self):

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

    def chose_room(self):

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

        return go_to


