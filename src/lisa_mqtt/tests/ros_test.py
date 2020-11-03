#!/usr/bin/env python3

import unittest
import subprocess
import sys
from time import sleep

import rosunit
import rosgraph
import rospy

from lisa_interaction_msgs.srv import UtterService, InteractService

TOPIC_RECOGNIZED = '/lisa/intent/'
TOPIC_NOT_RECOGNIZED = '/lisa/intent/not_recognized'
SERVICE_INTERACT = '/lisa/service/interact'
SERVICE_UTTER = '/lisa/service/say'

SCRIPT_NODE_NAME = 'run_roscore.bash'
SCRIPT_NODE_FOLDER = '/home/pi/sw/lisa_catkin_ws/src/lisa-mqtt-ros-bridge/scripts/'
LISA_MQTT_NODE = ["lisa-mqtt-ros-bridge", "bridge-node.py"]

# ------------
# --- TEST ---
# ------------ 
class TestLisaRosInteraction(unittest.TestCase):

	@classmethod
	def setUp(cls):
		# def _run_mqtt_node():
			# cls.mqtt_bridge_p = subprocess.Popen(["rosrun", LISA_MQTT_NODE[0], LISA_MQTT_NODE[1]], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
			# sleep(3)
	
		if not rosgraph.is_master_online():
			print("Run ROS core")
			#cls.kill_core_on_exit = False
			#_run_mqtt_node()
			raise Exception("ROS core not available")
		# TODO: check mqtt broker
		
		
		# print("Starting ROS Core")
		# script_src = SCRIPT_NODE_FOLDER + SCRIPT_NODE_NAME
		
		# # cls.roscore_p = subprocess.Popen([sys.executable, '-c', script_src], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
		# cls.roscore_p = subprocess.Popen(["/bin/bash", script_src], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
		
		# _check = 0
		# while True:
			# if rosgraph.is_master_online(): # Checks the master uri and results boolean (True or False)
				# break
			# else:
				# _check += 1
			# if _check > 10:
				# raise Exception("Cannot connect to ROS Master")
			# sleep(1.5)
		# _run_mqtt_node()

	@classmethod
	def tearDownClass(cls):
		pass
		# if cls.kill_core_on_exit:
			# print("Killing Core")
			# cls.roscore_p.kill()
			# subprocess.Popen(["killall" , SCRIPT_NODE_NAME, "roscore", "rosmaster", "rosout"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
		# if cls.mqtt_bridge_p is not None:
			# cls.mqtt_bridge_p.kill()
		# subprocess.Popen(["pgrep", "-f", LISA_MQTT_NODE[1], "|", "xargs", "kill"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

	def test_check_services_available(self):
		rospy.wait_for_service(SERVICE_INTERACT, timeout=3)
		rospy.wait_for_service(SERVICE_UTTER, timeout=3)
		utter = rospy.ServiceProxy(SERVICE_UTTER, UtterService)
		interact = rospy.ServiceProxy(SERVICE_INTERACT, InteractService)

	def test_call_utter_service_single(self):
		# dataset
		context_1 = "" #"Context_1"
		text_1 = "Utter text in context one"
		canBeEnqued_1 = False

		# connect to service
		rospy.wait_for_service(SERVICE_INTERACT, timeout=3)		
		utter = rospy.ServiceProxy(SERVICE_UTTER, UtterService)
				
		# call service
		utter_response = utter(context_id=context_1, text=text_1, canbeenqued=canBeEnqued_1)
		
		# analyse response
		print("Utter response is: ", utter_response)
		
	def test_check_topics_available(self):
		# 
		pass
		
		

if __name__ == '__main__':
	rosunit.unitrun("lisa_mqtt_ros_bridge", "test_ros_interaction", TestLisaRosInteraction)