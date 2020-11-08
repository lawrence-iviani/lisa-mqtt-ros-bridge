#!/usr/bin/env python3

import unittest
import subprocess
import sys
from time import sleep, time

import rosunit
import rosgraph
import rospy

from lisa_interaction_msgs.srv import UtterService, InteractService
from lisa_interaction_msgs.msg import IntentMessage, IntentNotRecognizedMessage, TtsSessionEnded
from lisa_mqtt.tests.ut_sm_test import test_case_str


TOPIC_INTENT_RECOGNIZED = '/lisa/intent/'
TOPIC_NOT_RECOGNIZED = '/lisa/intent/not_recognized'
TOPIC_TTS_FINISHED = '/lisa/tts/finished'
SERVICE_INTERACT = '/lisa/service/interact'
SERVICE_UTTER = '/lisa/service/say'

# ------------
# --- TEST ---
# ------------
class TestLisaRosInteraction(unittest.TestCase):

	@classmethod
	def setUp(cls):
		if not rosgraph.is_master_online():
			print("Run ROS core")
			#cls.kill_core_on_exit = False
			#_run_mqtt_node()
			raise Exception("ROS core not available")
		rospy.init_node('TestLisaRosInteraction')
		# TODO: check mqtt broker

	@classmethod
	def tearDownClass(cls):
		pass

	def test_check_topics_available(self):
		def _on_intent_received():
			pass	
		intent = rospy.Subscriber(TOPIC_INTENT_RECOGNIZED, IntentMessage, _on_intent_received)		
		intent_not_recognized = rospy.Subscriber(TOPIC_NOT_RECOGNIZED, IntentNotRecognizedMessage, _on_intent_received)		
		sleep(0.5)
		intent.unregister()
		intent_not_recognized.unregister()

	def test_check_services_available(self):
		rospy.wait_for_service(SERVICE_INTERACT, timeout=3)
		rospy.wait_for_service(SERVICE_UTTER, timeout=3)
		utter = rospy.ServiceProxy(SERVICE_UTTER, UtterService)
		interact = rospy.ServiceProxy(SERVICE_INTERACT, InteractService)

	def test_call_interact_service_single(self):		
		print(test_case_str('*** Start test_call_interact_service_single ***'))
	
		# dataset
		context_1 = ["Context_A",None]
		text_1 = "Tell me something "
		canBeEnqued_1 = [True, False]
		canBeDiscarded_1  = [True, False]
		wait_time = 5
		wait_topics = {TOPIC_INTENT_RECOGNIZED: IntentMessage, TOPIC_NOT_RECOGNIZED: IntentNotRecognizedMessage}
		wait_answer = True
		answered, not_answered, not_executed = self._run_single_call(
									SERVICE_INTERACT, InteractService, context_1, canBeEnqued_1, canBeDiscarded_1,
									text_to_utter=text_1, wait_topics=wait_topics, wait_time=wait_time, wait_answer=wait_answer)

		print(test_case_str("test_call_interact_service_single results:"))
		print("-----\nAnswered {}\n{}".format(len(answered), answered))
		print("-----\nNot Answered {}\n{}".format(len(not_answered), not_answered))
		print("-----\nNot Executed {}\n{}".format(len(not_executed), not_executed))
		return answered, not_answered, not_executed
		
	def test_call_utter_service_single(self):	
		print(test_case_str('*** Start test_call_utter_service_single ***'))
	
		# dataset
		context_1 = ["Context_A",None]
		text_1 = "I say something "
		canBeEnqued_1 = [True, False]
		canBeDiscarded_1 = [True, False]
		wait_topics = {TOPIC_TTS_FINISHED: TtsSessionEnded}
		wait_time = 3
		wait_answer = True
		
		answered, not_answered, not_executed = self._run_single_call(
								SERVICE_UTTER, UtterService, context_1, canBeEnqued_1, canBeDiscarded_1,
								text_to_utter=text_1, wait_topics=wait_topics, wait_time=wait_time, wait_answer=wait_answer)
		print(test_case_str("test_call_utter_service_single results:"))
		print("-----\nAnswered {}\n{}".format(len(answered), answered))
		print("-----\nNot Answered {}\n{}".format(len(not_answered), not_answered))
		print("-----\nNot Executed {}\n{}".format(len(not_executed), not_executed))
		return answered, not_answered, not_executed
	
	def _run_single_call(self, service, service_type, context_id, canBeEnqued, canBeDiscarded,
						 text_to_utter='', wait_topics={}, wait_time=0.25, wait_answer=False):
		"""
		Utter context_id and canBeEnqued by callint the service_proxy
		"""
		import threading 
		from collections import namedtuple
		Result = namedtuple('Result', ['n_test', 'context_id', 'canBeEnqued', 'canBeDiscarded'])
		
		if not isinstance(context_id, list):
			context_id = [context_id]
		if not isinstance(canBeEnqued, list):
			canBeEnqued = [canBeEnqued]
		if not isinstance(canBeDiscarded, list):
			canBeDiscarded = [canBeDiscarded]
		if wait_answer and not (isinstance(wait_topics, dict) and len(wait_topics)):
			print('wait_topics='+str(wait_topics))
			print('len(wait_topics)='+str(len(wait_topics)))
			assert False, "Invalid option for waiting an answer expected a dictonary with at least one key. {}".format(wait_topics)
		
		rospy.wait_for_service(service, timeout=wait_time)		
		service_proxy = rospy.ServiceProxy(service, service_type)
		
		context_called = [None,]
		topic_called = [False,]
		
		if wait_answer:
			def _called(data):
				print('\t\t\t---CALLBACK: data: ' + str(data))
				context_called[0] = data.context_id
		
			def _wait_answer(topic_called, topic, topic_type, n_test):
				print("\t\tTHREAD-----TEST({}): Calling {} called type {} ".format(n_test, topic, topic_type))
				try:
					rospy.wait_for_message(topic, topic_type, timeout=wait_time)
					print("\t\tTHREAD-----TEST({}):  topic: {} - {}".format(n_test, topic, topic_type))
					topic_called[0] = True
				except rospy.exceptions.ROSException as re:
					print("\t\tTHREAD-----TEST({}): EXCEPTION for topic: {} - {} - {}".format(n_test, topic, topic_type, re))
					topic_called[0] = False
				# print("\t\tTHREAD-----TEST({}): topic_called is {} value is {}".format(n_test, id(topic_called), topic_called[0]))
			topics = [rospy.Subscriber(t[0], t[1], _called) for t in zip(wait_topics.keys(), wait_topics.values())]
			print("wait_answer: listen to topics: " + str(topics))
							
		# call service
		answered = []
		not_answered = []
		not_executed = []
		n_test = 0
		for n, c in enumerate(context_id):
			for nn, e in enumerate(canBeEnqued):
				for nnn, d in enumerate(canBeDiscarded):
					n_test += 1
					_context = c if c is None else str(c)+"_"+str(n_test)
					print('[{}] Calling: context={} canBeEnqued={} canBeDiscarded={}'.format(n_test, _context, e, d), end='...\t\t')
					response = service_proxy(context_id=_context, text=text_to_utter+str(n_test), canbeenqued=e, canbediscarded=d).success
					print('Response: ' + str(response))
					if not response:
						not_executed.append(Result(n_test, _context, e, d))
						continue
					if wait_answer:
						thrd = []
						for t in zip(wait_topics.keys(), wait_topics.values()):
							th = threading.Thread(target=_wait_answer, args=(topic_called,t[0],t[1],n_test))
							th_name = str(t[1])
							th_name = th_name.split('\'')[1].split('.')[-1]#.split('\'')[1].split('.')[-1]
							th.name = "listen_{}_{}_{}_{}".format(_context, e, d, th_name)
							print('[{}] start thread: {}'.format(n_test, th.name))
							th.start()
							thrd.append(th)
						_finished = False
						while(not _finished):
							thread_finished = []
							for t in thrd:
								_finished = _finished or not t.is_alive()
								if not t.is_alive():
									thread_finished.append("Thread ended: " + t.name + "...\t")
							sleep(0.2)
						print('[{}] {} thread(s) Finished - Topic called {} - finished: {}'.format(n_test, len(thread_finished),topic_called[0]," - ".join(thread_finished)))
						#print("topic_called is {} value is {}".format(id(topic_called), topic_called[0]))
						if topic_called[0]:
							answered.append(Result(n_test, _context, e, d))
							assert context_called[0] is not None, "Results available but context is not for test_id[{}], "\
																  "context={} canBeEnqued={} canBeDiscarded={}".format(n_test,
																   _context, e, d)
							assert context_called[0] == _context, 'For test_id[{}] context fail: original={} vs {}=called with '\
																  'canBeEnqued={} canBeDiscarded={}'.format(n_test,
																   _context, context_called[0],  e, d)
							# if context_called[0] is not None:
								# # if context_called[0]==
							print('[{}] context: original={} SAME of {}=called '.format(n_test, _context, context_called[0]))
							# else:
								# print('[{}] !!! context_called should be not None, original {}'.format(n_test, _context))
						else:
							not_answered.append(Result(n_test, _context, e, d))
						# assert topic_called, "Topic answer not received for case {}_{}_{}".format(_context, e, d)
						topic_called[0] = False
						
							
						context_called[0] = None
					else:	
						answered.append(Result(n_test, _context, e, d))
						sleep(wait_time)
					print("-----------\n")
		return answered, not_answered, not_executed
	
if __name__ == '__main__':
	rosunit.unitrun("lisa_mqtt_ros_bridge", "test_ros_interaction", TestLisaRosInteraction)