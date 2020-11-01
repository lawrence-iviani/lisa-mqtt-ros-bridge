#!/usr/bin/env python3

import threading
from time import sleep

## For ROS
# Intent Detection
import rospy
# from std_msgs.msg import String

# Talk Service
import rospy

# For MQTT
#import json
import argparse

# pip install paho-mqtt
import paho.mqtt.client as mqtt

from lisa_mqtt.dialogue import DialogueManager
from lisa_mqtt.sound_sources import SoundSourcesManager


MQTT_PORT = 12183 # MQTT Broker Port
USE_INTENT_SERVICE = True



# Shared structures
managers = [] 
topic_dict = {}
topic_broadcast_dict = {}


# MQTT FUNCTIONS
def on_connect(client, userdata, flags, rc):

	for m in managers:
		#print('---------- MQTT Manager: ', m)
		topic_list = m.mqtt_topics_to_subscribe
		#print('on_connect->topic_list=', topic_list)
		for k,v in topic_list.items():
			#print('\t subscribing: ', k ,v)
			client.subscribe(k)
			if '#' in k:
				topic_broadcast_dict[k] = v
			else:
				topic_dict[k] = v

	rospy.loginfo("Connected. Waiting for subscribed MQTT topics.")("Connected. Waiting for subscribed MQTT topics.")

def _is_topic_in_broadcast_key(topic_key):
	for k in topic_broadcast_dict.keys():
		topic = k.split('#')[0]
		if topic in topic_key:
			return True
	return False

def on_disconnect(client, userdata, flags, rc):
	"""Called when disconnected from MQTT broker."""
	client.reconnect()


def on_message(client, userdata, msg):
	"""Called each time a message is received on a subscribed topic."""
	try:
		_executed = False
		if msg.topic in topic_dict.keys():
			topic = topic_dict[msg.topic]
			#print('calling: ', topic )
			topic.function_handler(topic.specific_topic, client, userdata, msg)
			_executed = True
		elif  _is_topic_in_broadcast_key(msg.topic):
			for k, v in topic_broadcast_dict.items():
				t = k.split('#')[0]
				# check if the broadcast topic is present
				if not(t in msg.topic):
					continue
				topic = topic_broadcast_dict[k]
				# print('calling broadcast: ', topic )
				specific_topic = msg.topic[len(t):] # shoudl be only the last part??
				topic.function_handler(specific_topic, client, userdata, msg)
				_executed = True
				break
		if not _executed:
			rospy.logwarn('Topic [{}] not registered, doing nothing'.format(msg.topic))
	except Exception as e:
		rospy.logwarn('on message: {} - {}'.format(msg.topic, e))
    

def setup_managers(client, level_str):
	rospy.loginfo("setup_managers ")
	# TODO: this should be automatized and configurable
	managers.append(DialogueManager(client, args))
	managers.append(SoundSourcesManager(client, args))
	# TODO: update the intent service flag from args before setting up ros
	# args.call_intent_service
	for m in managers:
		# print('---------- ROS services: ', m)
		#m.set_client(client)
		m.init_ros_topics()
	
	
def get_debug_level(args):
	if args == 'debug':
		return rospy.DEBUG
	elif args == 'info':
		return rospy.INFO
	elif args == 'warn':
		return rospy.WARN 
	elif args == 'error':
		return rospy.ERROR 
	elif args == 'fatal':
		return rospy.FATAL 
	else:
		return rospy.INFO

	
def mqtt_client(args):
    # Create MQTT client and connect to broker
	rospy.loginfo("Setting up MQTT Client")
	client = mqtt.Client()
	client.on_connect = on_connect
	client.on_disconnect = on_disconnect
	client.on_message = on_message

	rospy.loginfo("Setting up ROS node")
	setup_managers(client, args)

	rospy.loginfo("Connectiong MQTT Client")
	client.connect(args.host, args.port)

	try:
		rospy.loginfo("Starting MQTT Client")
		client.loop_forever()
	except KeyboardInterrupt:
		pass
	finally:
		# _LOGGER.debug("Shutting down")
		rospy.loginfo("Shutting down MQTT Client")


if __name__ == '__main__':
	# Start MQTT client
	"""Main method."""
	parser = argparse.ArgumentParser(prog="rhasspy-other-brokers-cli-hermes")
	parser.add_argument(
		"--port", default=MQTT_PORT, help="MQTT Broker Port, default is " + str(MQTT_PORT)
	)
	parser.add_argument(
		"--host",
		default="127.0.0.1",
		help="MQTT Host (default: 127.0.0.1)",
	)
	parser.add_argument(
		"--call_intent_service",
		action='store_const', const=True,
		default=False,  # or MAX_ODAS_SOURCES,
		help="An intent is called as service " + "" + "intent_name, instead a topic is published at the address"
	)
	parser.add_argument(
		"--debug",
		#action='store_const', const=True, 
		choices=['debug', 'info', 'warn', 'error', 'fatal'],
		default='debug',  # TODO: change in info
		help="set the debug level of the node"
	)
	
	args = parser.parse_args()
	dbg_level = get_debug_level(args.debug)
	
	# start first the rospy node
	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('lisa_mqtt_ros_bridge', anonymous=True, log_level=dbg_level)

	# start MQTT listener client
	threading.Thread(target=mqtt_client, args=(args,), daemon=True).start()

	# keeps python from exiting until this node is stopped
	sleep(1)
	rospy.loginfo("Starting ROS node")
	try:
		rospy.spin()
	except Excpetion as e:
		rospy.logwarn("Catched  ROS exception " + str(e))
	rospy.loginfo("Ending ROS node and main")





