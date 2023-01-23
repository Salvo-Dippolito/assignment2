#!/usr/bin/env python

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
import numpy as np

from ontology_interface import HandleOntology

class ReadRoomCreateOntology:

	"""

	"""
	def __init__(self):

		self.pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
		

		#initializing ontology
		#-------------------------------------------------------------------------------
		self.client = ArmorClient('surveyor_','map_ontology_')
		self.locations_list=[]
		self.doors_list=[]

		self.path = dirname(realpath(__file__)) + '/../ontologies/topological_map.owl'
		self.client.utils.load_ref_from_file(self.path, 'http://bnc/exp-rob-lab/2022-23', True, 'PELLET', True, False)
		self.client.utils.mount_on_ref()
		#-------------------------------------------------------------------------------

		self.configurations=[ [0,0.8,-0.4,0,-0.8],[0,1.5,-0.4,0,-0.8],[-1.57,1.2,-0.4,0.2,-1],[1.35,1.7,-0.4,0,-0.8],[3.2,0.1,-0.2,0,-0.3],[3,0.3,0.4,0,-0.3],[0.55,1.7,-0.4,0,-0.8]]
		self.current_configuration=[0,0,0,0,0]


		# This is used for reference when printing on screen:
		self.LINE=150

	def read_room_create_ontology(self):

		print("\n============================================================\n")

		self.move_to_joint_angles([0,0,0,0,0])

		room_coordinates_dict={}
		for configuration in self.configurations:
			self.current_configuration=configuration
			self.move_to_joint_angles(configuration)
			room,coordinates=self.get_room_layout_service_client()
			room_coordinates_dict[room]=coordinates

		print(room_coordinates_dict)

		self.old_value = self.client.query.dataprop_b2_ind('urgencyThreshold','Robot1')
		self.old_value = self.old_value[0]
		self.old_value = self.old_value[1:-11]

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
	
	def add_location(self, name_of_location):
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

	def connect_locations(self, location1,location2,door_name):
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

	def get_number_of(self, what):

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
		rospy.wait_for_service('/get_marker_id')
		try:
			service_handle = rospy.ServiceProxy('/get_marker_id', Trigger)
			req = TriggerRequest()
			res = service_handle(req)

			while not res.success:
				res = service_handle(req)
				print("hello there ")
				print(len(res.message))
				print("server response is: \n")
				print(res)
				self.giggle( self.current_configuration)
			
			print("read marker id: "+str(res.message))
			return res.message
		except rospy.ServiceException as e:

			print("Service call failed: %s" % e)

	def get_room_layout_service_client(self):
	   	rospy.wait_for_service('/room_info')

	   	try:
	   		service_handle = rospy.ServiceProxy('/room_info', RoomInformation)

	   		req= RoomInformationRequest()
	   		req.id=int(self.marker_id_service_client())
	   		res = service_handle(req)
	   		print("adding location "+res.room)
	   		self.add_location(res.room)

	   		for connection in res.connections:
	   			print("connecting location "+res.room+" with location "+connection.connected_to +" with door name "+connection.through_door)
	   			self.connect_locations(res.room,connection.connected_to, connection.through_door)
	   		print("\n============================================================\n")

	   		return res.room, [res.x,res.y]

	   	except rospy.ServiceException as e:
	   		print("Service call failed: %s" % e)

	def move_to_joint_angles(self,c):
		msg = JointTrajectory()
		msg.joint_names = ["arm_base_joint","shoulder_joint", "bottom_wrist_joint", "elbow_joint","top_wrist_joint"]
		point = JointTrajectoryPoint()
		point.positions = [c[0],c[1],c[2],c[3],c[4]]
		point.time_from_start = rospy.Duration(1)
		msg.header.stamp = rospy.Time.now()
		msg.points.append(point)
		self.pub.publish(msg)
		rospy.sleep(2)
		

	def giggle(self,old_configuration):
		
		new_config= old_configuration+ np.random.uniform(-0.1, 0.1, 5)
		self.move_to_joint_angles( new_config)
		print ("moved to ")
		print(new_config)





if __name__ == '__main__':

	rospy.init_node('joint_trajectory_publisher')

	pippo=ReadRoomCreateOntology()
	print(pippo.read_room_create_ontology())
	
	handle_ontology=HandleOntology()
	print(handle_ontology.read_room_create_ontology())
