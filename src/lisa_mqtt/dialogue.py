
from collections import OrderedDict
import json

# State machine pytransitions
from transitions import Machine, State
from transitions.core import MachineError

# ROS Stuff
import rospy
import actionlib

# ROS LISA 
from lisa_interaction_msgs.srv import IntentService
from lisa_interaction_msgs.srv import UtterService, UtterServiceResponse
from lisa_interaction_msgs.srv import InteractService, InteractServiceResponse
from lisa_interaction_msgs.msg import IntentMessage, IntentNotRecognizedMessage, TtsSessionEnded
# TTS actionlib messages
from lisa_interaction_msgs.msg import LisaUtterAction, LisaUtterFeedback, LisaUtterResult

# local functions
from . import _print_debug, _print_status, _print_topic_data, FixSizeOrderedDict, Topic, time_ref, current_milli_time, ROS_SERVICE_PREFIX, ManagerInterface

# ---------------------
# --- CONFIGURATION ---
# --------------------- 

# here to add the handler information e.g self.handle_asr self.handle_dialogue etc
subscribe_topics = {'hermes/dialogueManager/': ['sessionStarted', 'sessionEnded', 'sessionQueued', 'intentNotRecognized', 'startSession','continueSession', 'endSession'],
					'hermes/hotword/':         ['toggleOn', 'toggleOff', 'default/detected'],
					'hermes/asr/':             ['toggleOn', 'toggleOff', 'startListening', 'stopListening', 'partialTextCaptured', 'textCaptured'],
					'hermes/nlu/':             ['query', 'partialQuery', 'intentParsed', 'slotParsed', 'intentNotRecognized'],
					'hermes/intent/':          ['#'],
					'hermes/error/':           ['nlu'],
					'hermes/tts/':             ['say', 'sayFinished']}


# Defne transitions
states=[State(name='WaitingSession',),# on_exit=['say_goodbye']),
		# State(name='ClosingSession',),# on_exit=['say_hello']),
		State(name='WaitingIntent',),# on_exit=['say_hello']),
		State(name='Uttering',),]# on_exit=['say_hello']),]


# Defne transitions
transitions = [
   # Session
	{ 'trigger': 'start_session', 			'source': 'WaitingSession', 'dest': 'WaitingIntent' ,	'before': 'reset' },
	{ 'trigger': 'close_session', 			'source': 'WaitingIntent', 	'dest': 'WaitingSession',	'before': 'reset' },
	{ 'trigger': 'reset_session',   		'source': 'WaitingIntent',  'dest': 'WaitingSession', 	'before': 'reset_rhasspy_session' },

	# Uttering
	{ 'trigger': 'utter', 					'source': 'WaitingIntent', 	'dest': 'Uttering' },
	{ 'trigger': 'uttered', 				'source': 'Uttering', 		'dest': 'WaitingIntent' },

	# Intentions
	{ 'trigger': 'intent_recognized',		'source': 'WaitingIntent', 	'dest': 'WaitingIntent', 	'before': 'add_action' },
	{ 'trigger': 'intent_not_recognized',	'source': 'WaitingIntent', 	'dest': 'WaitingIntent', 	'before': 'add_action'}, # storing also the non recognized, perhaps there is a meaningful input
]

ANONYMOUS_PREFIX = 'Anonymous_'

# ------------
# --- CODE ---
# ------------ 

# class ExternalContextSession:
	# MAX_CONTEXT_LENGTH = 10  # the max length of action/interaction session (this the mimimum element??)

	# # possible states
	# INVALID = -1
	# ACTIVE=0
	# SUSPENDED=1
	# UTTERING=2

	# _list_rh_sessions = None

	# def __init__(self, client_id):
		# self.client_id=client_id
		# self._list_rh_sessions=FixSizeOrderedDict(max=self.MAX_CONTEXT_LENGTH)
		# self.state=self.INVALID

	# def add_rhasspy_session(rhasspy_session):
		# if rhasspy_session is None:
			# return False
		# self._list_rh_sessions[rhasspy_session.session] = rhasspy_session
		# return True


class DialogueException(Exception):
	pass


class DialogueSession:
	_active_rhasspy_session = None
	_site_rhasspy_id = ''
	time_start = None
	session_data = None
	actions_list = []
	_anonymous_id = 0

	def __init__(self):
		self.reset()

	def reset(self, **kwargs):
		rospy.logdebug("DialogueSession: reset session={}, rh_session_id={} after {} ms".format(
			self.session_data, self._active_rhasspy_session, self.elasped_time))
		self._active_rhasspy_session = None
		self._time_start  = None
		self._site_rhasspy_id = None
		self._session_data = None
		self.actions_list = []

	def reset_rhasspy_session(self, **kwargs):
		rospy.logdebug("DialogueSession: reset rhasspy session {} after {} ms".format(self._active_rhasspy_session, self.elasped_time))
		self._active_rhasspy_session = None
		self._site_rhasspy_id = None

	def on_enter_WaitingIntent(self, rh_session_id=None, rh_site_id=None, session_data=None, action=None):#,rh_session_id = None, rh_site_id = None, session_data = None):
		rospy.logdebug("on_enter_WaitingIntent: rh_session_id={} rh_site_id={} session_data={}".format(self._active_rhasspy_session, self._site_rhasspy_id, self._session_data))
		# TODO: shoud assert  rh_session_id==self._active_rhasspy_session and so for the others?
		#
		# is it still the same session?
		# or is a clean up?

	def on_exit_WaitingIntent(self, rh_session_id=None, rh_site_id=None, session_data=None, action=None):
		# register any detected action
		rospy.logdebug("on_exit_WaitingIntent: rh_session_id={} rh_site_id={} session_data={}".format(self._active_rhasspy_session, self._site_rhasspy_id, self._session_data))

		# if action is not None:
			# rospy.loginfo('Session %s, rh_sess_id=%s,  Adding action: %s ', str(session_data), str(rh_session_id) ,str(action))
			# TODO:register action
		# elif  rh_session_id is not None and rh_site_id is not None and rh_site_id is not None:
			# rospy.loginfo('Action none for Session %s, rh_sess_id=%s,  Adding action: %s ', str(session_data), str(rh_session_id) ,str(action))

	def on_enter_WaitingSession(self, **kwargs):
		rospy.logdebug("on_enter_WaitingSession: rh_session_id={} rh_site_id={} session_data={}".format(self._active_rhasspy_session, self._site_rhasspy_id, self._session_data))

		if self._active_rhasspy_session is not None:
			pass#rospy.logdebug("on_exit_WaitingSession: rh_session_id={} rh_site_id={} session_data={}".format(rh_session_id, rh_site_id, session_data))

	def on_exit_WaitingSession(self,rh_session_id, rh_site_id, session_data = None):
		# Call exiting from the status, this init the new session
		rospy.logdebug("on_exit_WaitingSession: rh_session_id={} rh_site_id={} session_data={}".format(rh_session_id, rh_site_id, session_data))

		if self.has_active_rhasspy_session():
			raise DialogueException('Rhasspy session cannot be none')

		if self.has_session_data():
			# Follow cases
			if session_data is None:
				# i have a valid session but an anonymous session is requested, there should be a priority??
				# rospy.logdebug("on_exit_WaitingSession: request an anoymous session with existing session data: %s", self.session)
				# archive session, actions etc
				rospy.logdebug("on_exit_WaitingSession: call reset with session data: %s", session_data)
				self.reset() # is it safe? i wanna clear the status
			elif session_data == self.session:
				rospy.logdebug("on_exit_WaitingSession: an existing external session to continue: %s", session_data)
				# update only the
				self._active_rhasspy_session = rh_session_id
				self._site_rhasspy_id = rh_site_id
				return # is it safe? i wanna clear the status
			else:
				rospy.logdebug("on_exit_WaitingSession: a new external session to continue: %s", session_data)
				self.reset()

		# initiate the session
		if session_data is None:
			self._anonymous_id += 1
			session_data = ANONYMOUS_PREFIX + str(self._anonymous_id)
			rospy.logdebug("on_exit_WaitingSession: starting anonymous session: {}".format(session_data))

		rospy.logdebug("DialogueSession: starting new dialogue={} rh_sess_id={}, rh_site_id={} at {} ms".format(session_data, rh_session_id, rh_site_id, current_milli_time() ))
		# assumption, no previous session are running...
		assert self._active_rhasspy_session is None and self._session_data is None and len(self.actions_list)==0, "Cannot create new session if previous was not reset"
		self._active_rhasspy_session = rh_session_id
		self._site_rhasspy_id = rh_site_id
		self._session_data = session_data
		self._time_start  = current_milli_time()

	def has_active_rhasspy_session(self):
		return not self.rhasspy_session == None

	def is_active_rhasspy_session(self, rh_session_id):
		return self.rhasspy_session == rh_session_id

	def is_valid_rh_session(self, rh_session_id, rh_site_id):
		return self.is_active_rhasspy_session(rh_session_id=rh_session_id) and self._site_rhasspy_id == rh_site_id

	def is_session_data(self, session_data):
		return self._session_data == session_data

	def has_session_data(self):
		return not self._session_data == None

	def get_session_from_rh_session(self, rh_session_id, rh_site_id):
		if self.is_valid_rh_session(rh_session_id, rh_site_id):
			return self.session
		else:
			return None

	@property
	def actions(self):
		return self.actions

	@property
	def rhasspy_session(self):
		return self._active_rhasspy_session

	@rhasspy_session.setter
	def rhasspy_session(self, new_rhasspy_session_id):
		self._active_rhasspy_session  = new_rhasspy_session_id

	@property
	def rhasspy_site(self):
		return self._site_rhasspy_id if self.has_active_rhasspy_session() else None

	@property
	def session(self):
		return self._session_data # if self.has_active_session() else None

	@property
	def site(self):
		return self._site_rhasspy_id if self.has_active_rhasspy_session() else None

	@property
	def elasped_time(self):
		return current_milli_time()-self._time_start if self.has_active_rhasspy_session() else None

	@property
	def start_time(self):
		return self._time_start if self.has_active_rhasspy_session() else None

	def add_action(self, rh_session_id, rh_site_id, session_data, action):
		assert isinstance(action, dict) and 'result' in action, 'Action wrong format or result key not available'

		if self.get_session_from_rh_session(rh_session_id, rh_site_id) == session_data:
			rospy.loginfo("add_action:  adding action={},".format(action ))
			self.actions_list.append((current_milli_time(), action))
			return True
		else:
			rospy.logwarn("add_action: doing nothing incompatible request(session={} or site{} or customData={}) actual session (session={}, rh_site_id={}, customData={})".format(
							rh_session_id,
							rh_site_id,
							custom_data,
							self._active_rhasspy_session,
							self._site_rhasspy_id,
							self.session))
			return False

	# TODO:next methods needs to reviewed.
	def get_session_info(self):
		"""
		If has an active session returns the rhasspy session id as defined in https://docs.snips.ai/reference/dialogue#session-started
		It records also start time etc.
		"""
		if self.has_active_rhasspy_session():
			return self._active_rhasspy_session, self._session_data, self._site_rhasspy_id, self._time_start, current_milli_time()-self._time_start
		else:
			return None


class DialogueManager(ManagerInterface):

	# _prog_number = 1
	# _session_dict = FixSizeOrderedDict(max=3)
	_USE_INTENT_SERVICE = False # a kind of backward compatibility... should be removed

	def __init__(self, mqtt_client, args):
		super().__init__(mqtt_client, args)
		self._USE_INTENT_SERVICE = args.call_intent_service
		self._session = DialogueSession()
		self._dialogue_sm = Machine(model=self._session, states=states, transitions=transitions, initial='WaitingSession')
		# self._dialogue_sm = DialogueStateMachine()

	def init_ros_topics(self):
		if self._USE_INTENT_SERVICE:
			raise NotImplementedError('Calling an intent service instead of publish the intent topic is not available')

		# what this node can tell
		self.pub_intent = rospy.Publisher(ROS_SERVICE_PREFIX + 'intent', IntentMessage, queue_size=10)
		self.pub_intent_not_rec = rospy.Publisher(ROS_SERVICE_PREFIX + 'intent/not_recognized', IntentNotRecognizedMessage, queue_size=10)
		self.pub_tts_finished = rospy.Publisher(ROS_SERVICE_PREFIX + 'tts/finished', TtsSessionEnded, queue_size=10)

		# start utter service listening on ROS and using MQTT client to call services
		self.action_utter = actionlib.SimpleActionServer('/lisa/say', LisaUtterAction, self.execute_utter_actionlib_callback, False)
		self.action_utter.start()

		self.srv_say = rospy.Service('lisa/service/say', UtterService, self.execute_utter_ros_service)
		self.srv_interact = rospy.Service('lisa/service/interact', InteractService, self.execute_interaction_ros_service)

	def execute_utter_ros_service(self, request):
		#response = InteractServiceResponse()
		try:
			retval = self._perform_interaction(request, utter_only=True)
			rospy.logdebug("executed utter only with result: {}".format(retval))
		except Exception as e:
			rospy.logdebug('Catched excpetion exceuting uttering: {}'.format(e))
			retval = False
		return retval
	
	def execute_interaction_ros_service(self, request):
		#response = InteractServiceResponse()
		try:
			retval = self._perform_interaction(request, utter_only=False)
			rospy.logdebug("executed inteaction with result: {}".format(retval))
		except Exception as e:
			rospy.logdebug('Catched excpetion exceuting interaction: {}'.format(e))
			retval = False
		return retval

	def _perform_interaction(self, request, utter_only=False):
		# print('execute_interaction_ros_service',request)
		context_id = request.context_id

		if utter_only:
			interaction_type='notification'
		else:
			interaction_type='action'
		if context_id is not None and len(context_id):
			rospy.logdebug("Recieved ROS request for interaction {} with context_id {}".format(interaction_type, context_id))
			retval = self.direct_interaction(type=interaction_type, text=request.text, canBeEnqueued=request.canbeenqued,
											 canBeDiscarded=request.canbediscarded ,customData=context_id)
			print("execute_interaction_ros_service with context retval=", retval)
			response.success = retval
		else:
			rospy.logdebug("Recieved ROS request for interaction {} with anonymous context".format(interaction_type))
			retval = self.direct_interaction(type=interaction_type, text=request.text, canBeEnqueued=request.canbeenqued,
											 canBeDiscarded=request.canbediscarded )
			print("execute_interaction_ros_service NO context retval=", retval)
		return retval

	def execute_utter_actionlib_callback(self, goal_handle):
		sentence = goal_handle.sentence

		feedback_msg = LisaUtterFeedback()
		feedback_msg.percent_complete = 0.0

		result_msg = LisaUtterResult()

		# utter sentence
		# https://docs.snips.ai/reference/hermes#sending-text-to-be-spoken-by-the-tts-component-low-level-api
		# Note that this is a low-level API, and it is not recommended to be used of production. You should use the following topics instead, based on the dialogue manager:
		# hermes/dialogueManager/startSession
		# hermes/dialogueManager/continueSession
		# See the Dialogue API Reference for further explanations.
		# payload = TtsSay(text=sentence, lang="en_EN") # id=sentence for example?

		# instead of using TtsSay (complex for import and it is a json message), using some json magic
		#payload = json.dumps({"text": sentence})  # TtsSay(text=sentence, lang="en_EN")  # id=sentence for example?
		#rospy.loginfo("Received sentence to utter: {}".format(payload))

		rospy.loginfo("Received sentence to utter: {}".format(sentence))
		self.direct_interaction(type='notification', text=sentence, canBeEnqueued=False, canBeDiscarded=False)

		# TODO: mimic listening for an action, check for real progress instead of estimated
		SLEEP_TIME = 0.082 # TODO: MAGIC NUMBER HERE!!!!
		for i in range(0, len(sentence)):
			feedback_msg.percent_complete = float(i) / float(len(sentence))
			# self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
			self.action_utter.publish_feedback(feedback_msg)
			rospy.sleep(SLEEP_TIME)  # THE operation

		# TODO: this is bad, it needs a sync with the TTS system
		self.action_utter.set_succeeded(result_msg)

	# entry point for the dialog service
	def execute_new_dialogue_ros_service(self, request):
		pass
	def execute_end_dialogue_ros_service(self, request):
		pass

	@property
	def mqtt_topics_to_subscribe(self):
		retval = OrderedDict()
		# print('subscribe_topics.items()=',subscribe_topics.items())
		for k,v  in subscribe_topics.items():
			#print(k,v)
			if 'dialogueManager' in k:
				function_handler = self.handle_dialogue
			elif 'hotword' in k:
				function_handler = self.handle_hotword #(specific_topic, message)
			elif 'asr' in k:
				function_handler = self.handle_asr#(specific_topic, message)
			elif 'nlu' in k:
				function_handler = self.handle_nlu#(specific_topic, message)
			elif 'error' in k:
				function_handler = self.handle_error#(specific_topic, message)
			elif 'tts' in k:
				function_handler = self.handle_tts#(specific_topic, message)
			elif 'intent' in k:
				function_handler = self.handle_intents#(specific_topic, message)
			else:
				continue
			#print('v=',v)
			for t in v:
				general_topic = k
				specific_topic = t
				retval[k+t] = Topic(general_topic=k, specific_topic=t, function_handler=function_handler)
		#print('retval',retval)
		return retval
	# this should become a start section

	def direct_interaction(self, type='action', text='something to infer', canBeEnqueued=True, canBeDiscarded=True, customData=None):
		# siteId Optional String - Site where to start the session
		# init Object - Session initialization description: Action or Notification. See below
		# customData Optional String - Additional information that can be provided by the handler.
		# 	Each message related to the new session - sent by the Dialogue Manager - will contain this data

		# init Object
		# can_be_enqueued: bool  If true, the session will start when there is no pending one on this site. Otherwise, the session is just dropped if there is running one.
		# type: # DialogueActionType  'notification'|'action'
		# text: typing.Optional[str] = None   Text that the TTS should say at the beginning of the session.
		# intent_filter: typing.Optional[typing.List[str]] = None  A list of intents names to restrict the NLU resolution on the first query.
		# send_intent_not_recognized: bool = False
		# Indicates whether the dialogue manager should handle non-recognized intents by itself or send them for the client to handle.
		topic = "hermes/dialogueManager/startSession"
		assert customData is None or isinstance(customData,str), 'Custom data not supported: ' + str(customData)

		def _publish():
			init_pl = {"type": type, "text": text, "canBeEnqueued": canBeEnqueued}
			payload = json.dumps({"init": init_pl, 'customData': 	customData})
			# customData: this should be used when a client initate the session
			rospy.loginfo("publish MQTT topic {} with payload {}".format(topic, payload))
			self._mqtt_client.publish(topic=topic, payload=payload)

		if canBeEnqueued:
			_publish()
			retval = True
		elif self._session.has_active_rhasspy_session():
			if canBeDiscarded:
				rospy.loginfo("Previous session running {}, cannot publish MQTT topic {}".format(self._session.get_session_info(), topic))
				retval = False
			else:
				# TODO discard previous session or queue
				# sessionId:	String - Identifier of the session to end.
				# text :	 	Optional String - The text the TTS should say to end the session.
				session_id = self._session.rhasspy_session
				payload = json.dumps({"sessionId": session_id})
				rospy.loginfo("Previous session running {}, removing".format(self._session.get_session_info(), topic))
				self._mqtt_client.publish(topic="hermes/dialogueManager/endSession", payload=payload)
				_publish()
				retval = True
		else:
			_publish()
			retval = True
		return retval

	def handle_not_recognized_intents(self, specific_topic, client, userdata, message):

		# decode the message
		payload = json.loads(message.payload)
		# {'input': 'go move to conveyor', 'siteId': 'default', 'id': None, 'customData': None, 'sessionId': 'default-snowboy-d2a92308-90e6-4af0-af92-2206a44f9928'
		# get some info about the incoming topic
		input_str = payload["input"]
		site_id = payload["siteId"]
		session_id = payload['sessionId']
		custom_data = payload['customData']
		context_id = ''  # to check if is fine
		if self._session.is_valid_rh_session(rh_session_id=session_id, rh_site_id=site_id):
			context_id = self._session.session

		# todo:
		# if not custom_data==context_id

		# mapping to ROS
		# prepare for return message, this is how we map the MQTT intent not recognized in a ros message
		# string context_id
		# string original_input
		rospy.logdebug("The input ->{}<- was not recognized as a valid intent. RH_Session={}, site={}, context={} ".format(input_str,session_id,site_id,custom_data))
		msg = IntentNotRecognizedMessage(context_id=context_id, original_input=input_str)
		rospy.logdebug("publishing message: {} ".format(msg))
		self.pub_intent_not_rec.publish(msg)

	def handle_intents(self, specific_topic, client, userdata, message):
		# TODO: add the session context in the reply

		# decode the message
		payload = json.loads(message.payload)
		# get some info about the incoming topic
		message = message.topic
		intent_str = payload["intent"]
		input_str = payload["input"]
		slots = ["{}={}".format(slot['slotName'], slot['rawValue']) for slot in payload['slots']]
		site_id = payload["siteId"]
		context_id = ''  # to check if is fine
		if self._session.is_valid_rh_session(rh_session_id=payload['sessionId'], rh_site_id=payload['siteId']):
			context_id = self._session.session

		# mapping to ROS
		# prepare for return message, this is how we map the MQTT intent in a ros message
		# string intent_name
		# string[] pay_load
		# float32 confidence
		# TODO: add field -> string stt_input
		ros_intent_name = intent_str['intentName'] if 'intentName' in intent_str else topic[len("hermes/intent/"):]
		ros_pay_load = slots
		ros_confidence = intent_str['confidenceScore'] if 'confidenceScore' in intent_str else 0.0
		ros_input = input_str
		rospy.logdebug("->{}<- input, intent ->{}<- is coded as ROS message: {} - payload: {} - confidence: {} ".format(input_str, intent_str, ros_intent_name, ros_pay_load, ros_confidence))

		try:
			msg = IntentMessage(context_id=context_id, intent_name=ros_intent_name, pay_load=ros_pay_load,
								confidence=ros_confidence, original_input=ros_input)
			rospy.logdebug("publishing message: {} ".format(msg))
			self.pub_intent.publish(msg)
		except Exception as e:
			rospy.logwarn("Error publishing intent topic  {}: {}".format(ros_intent_name, e))

	# topic.specific_topic, client, userdata, msg
	def handle_dialogue(self, specific_topic, client, userdata, message):
		payload = json.loads(message.payload)
		parameters = ['sessionId', 'siteId', 'customData']
		_print_status('handle_dialogue', self._session, enter=True)
		if specific_topic == 'sessionStarted':
			# reset the timer
			time_ref  = current_milli_time()
			_print_topic_data('handle_dialogue', specific_topic, payload, parameters)
			# two cases, I have started the session OR the session was an interrupt from the user
			# a new session, I want to register it
			try:
				self._session.start_session(rh_session_id=payload['sessionId'], rh_site_id=payload['siteId'], session_data=payload['customData'] )
			except MachineError as e:
				rospy.logwarn('+-+-+-+-+-+-handle_dialogue:'+str(specific_topic)+': ERROR %s', str(e))
			except Exception as e:
				rospy.logerr('+-+-+-+-+-+-handle_dialogue:'+str(specific_topic)+': EXCEPTION %s', str(e))
				raise e
		elif specific_topic == 'sessionQueued':
			_print_topic_data('handle_dialogue', specific_topic, payload, parameters)
		elif specific_topic == 'sessionEnded':
			#_print_topic_data('handle_dialogue', specific_topic, payload, parameters + ['termination'])
			try:
				rospy.logdebug("+-+-+-+-+-+-Calling session_stopped_received: sessionId={} siteId={} termination={} customData={}".format(
					payload['sessionId'], payload['siteId'], payload['termination'], payload['customData']))
				# print("".join(["\tattribute: {} -> {}\n".format(k,v) for k,v in self._dialogue_sm.__dict__.items()]))# if 'session' in k])
				# def _continue():
					# TEST CODE, how do i continue a session???
					# payload_cmd = json.dumps({"sessionId": payload['sessionId'], "type": 'action', "canBeEnqueued": True, "customData": payload['customData']} )
					# print('continue session with payload=',payload_cmd)
					# customData: this should be used when a client initate the session
					# retval = self._mqtt_client.publish(topic="hermes/dialogueManager/continueSession", payload=payload_cmd)
					# _print_debug("continue_session: publish result {}".format(retval))
				termination_reason = payload['termination']['reason']
				print("+-+-+-+-+-+-REASON: ", termination_reason)
				if termination_reason == 'intentNotRecognized':
					# intentNotRecognized: the session ended because no intent was successfully detected.
					#_continue()
					self._session.reset_session(rh_session_id=payload['sessionId'], rh_site_id=payload['siteId'], session_data=payload['customData'])
				elif termination_reason == 'timeout':
					# timeout: The session timed out because no response from one of the components or no Continue Session or End Session from the handler code was received in a timely manner
					# _continue()
					self._session.reset_session(rh_session_id=payload['sessionId'], rh_site_id=payload['siteId'], session_data=payload['customData'])
				elif termination_reason == 'nominal':
					# nominal: the session ended as expected (a endSession message was received).
					# _continue()
					# this seems to be received after uttering
					self._session.reset_session(rh_session_id=payload['sessionId'], rh_site_id=payload['siteId'], session_data=payload['customData'])
				elif termination_reason == 'abortedByUser':
					# abortedByUser: the session aborted by the user.
					# _continue()
					# it should be receievd when you want to force delete a running session
					self._session.close_session(rh_session_id=payload['sessionId'], rh_site_id=payload['siteId'], session_data=payload['customData'])
				else:
					rospy.loginfo("+-+-+-+-+-+-REASON: UNMANAGED ", termination_reason)
					self._session.close_session(rh_session_id=payload['sessionId'], rh_site_id=payload['siteId'], session_data=payload['customData'])
			except MachineError as e:
				rospy.logwarn('+-+-+-+-+-+-handle_dialogue:'+str(specific_topic)+': ERROR %s', str(e))
			except Exception as e:
				rospy.logerr('+-+-+-+-+-+-handle_dialogue:'+str(specific_topic)+': EXCEPTION %s', str(e))
				raise e
		elif specific_topic == 'startSession':
			_print_topic_data('handle_dialogue', specific_topic, payload, None)
		elif specific_topic == 'continueSession':
			_print_topic_data('handle_dialogue', specific_topic, payload, None)
		else:
			print('this is the handle_dialogue, not suppoorted topic->',specific_topic)
		_print_status('handle_dialogue', self._session, enter=False)

	def handle_hotword(self, specific_topic, client, userdata, message):
		payload = json.loads(message.payload)
		if specific_topic == 'toggleOn':
			pass#_print_topic_data('handle_hotword', specific_topic, payload, ['siteId'])
			#print('toggleOn', payload)
			#_print_debug('handle_hotword->{} siteId={} '.format(specific_topic, payload['siteId']))
		elif specific_topic == 'toggleOff':
			pass#_print_topic_data('handle_hotword', specific_topic, payload, ['siteId'])
			#print('toggleOff', payload)
			#_print_debug('handle_hotword->{} siteId={}'.format(specific_topic, payload['siteId']))
		else:
			_print_debug('handle_hotword->{} +++UNHANDLED+++'.format(specific_topic))

	def handle_asr(self, specific_topic, client, userdata, message):
		payload = json.loads(message.payload)
		if specific_topic == 'startListening':
			pass
			#_print_topic_data('handle_asr-startListening', specific_topic, payload, None)
		elif specific_topic == 'stopListening':
			pass
			#_print_topic_data('handle_asr-stopListening', specific_topic, payload, None)
		else:
			pass
			#_print_topic_data('handle_asr-other', specific_topic, payload, None)

	def handle_nlu(self, specific_topic, client, userdata, message):
		payload = json.loads(message.payload)
		parameters = ['sessionId', 'siteId', 'input']
		_print_status('handle_nlu', self._session, enter=True)
		if specific_topic == 'intentParsed':
			try:
				# Note: every single intent is recorded and a message automatically sent
				_print_topic_data('handle_nlu', specific_topic, payload, parameters + ['intent'])
				action = payload['intent']
				action['input'] = payload['input']
				action['slots'] = payload['slots']
				action['result'] = specific_topic
				# TODO: This is a bad pattern, session_data has to be retrieved and passed to the transition. Couldn't be retrieved in the transition?
				#  this would decouple the rhasspy info in this class from the session...
				session_data=self._session.get_session_from_rh_session(rh_session_id=payload['sessionId'], rh_site_id=payload['siteId'])
				self._session.intent_recognized( rh_session_id=payload['sessionId'], rh_site_id=payload['siteId'], session_data=session_data, action=action)
			except MachineError as e:
				rospy.logwarn('+-+-+-+-+-+-handle_nlu:'+str(specific_topic)+': ERROR %s', str(e))
			except Exception as e:
				rospy.logerr('+-+-+-+-+-+-handle_nlu:'+str(specific_topic)+': EXCEPTION %s', str(e))
				raise e
		elif specific_topic == 'intentNotRecognized':
			try:
				# when an intent is not recognized the mechanism for parsed intent is not in place
				# this has to be called manually
				_print_topic_data('handle_nlu', specific_topic, payload, parameters)
				self.handle_not_recognized_intents(specific_topic, client, userdata, message)

				# save non recognized as a action.
				action = {'input': payload['input'], 'result': specific_topic}
				# TODO: This is a bad pattern, session_data has to be retrieved and passed to the transition. Couldn't be retrieved in the transition?
				#  this would decouple the rhasspy info in this class from the session...
				session_data = self._session.get_session_from_rh_session(rh_session_id=payload['sessionId'], rh_site_id=payload['siteId'])
				self._session.intent_not_recognized(rh_session_id=payload['sessionId'], rh_site_id=payload['siteId'], session_data=session_data, action=action)
			except MachineError as e:
				rospy.logwarn('+-+-+-+-+-+-handle_nlu:'+str(specific_topic)+': ERROR %s', str(e))
			except Exception as e:
				rospy.logerr('+-+-+-+-+-+-handle_nlu:'+str(specific_topic)+': EXCEPTION %s', str(e))
				raise e
		else:
			_print_topic_data('handle_nlu not supported', specific_topic, payload, None)
		_print_status('handle_nlu', self._session, enter=False)

	def handle_error(self, specific_topic, client, userdata, message):
		print('this is the handle_error->',specific_topic)

	def handle_tts(self, specific_topic, client, userdata, message):
		payload = json.loads(message.payload)
		session_data = self._session.get_session_from_rh_session(rh_session_id=payload['sessionId'], rh_site_id=payload['siteId'])
		msg = TtsSessionEnded(context_id=session_data, ) # todo:  text_uttered=message.????) to add text uttered
		rospy.logdebug("publishing message: {} ".format(msg))
		self.pub_tts_finished.publish(msg)

		print_topic_data('handle_tts', specific_topic, payload, None)


