
from collections import OrderedDict
import json
from sys import stderr

# State machine pytransitions
from transitions import Machine, State
from transitions.core import MachineError

# ROS Stuff
import rospy
import actionlib

# ROS LISA 
# from lisa_interaction_msgs.srv import IntentService
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

HOTWORD = 'snowboy' # TODO: should be a list and configurable at startup
DEFAULT_RH_SITE = 'default' # TODO: should be configurable at startup
INIT_STATE = 'Init'
ANONYMOUS_PREFIX = 'Anonymous_'
WAKEUPWORD_PREFIX = 'Wakeupword_'


# MQTT topics to listen
# Wakeup word, detection message is hermes/hotword/<WAKEWORD_ID>/detected
# https://rhasspy.readthedocs.io/en/latest/services/#wake-word-detection
subscribe_topics = {'hermes/dialogueManager/': ['sessionStarted', 'sessionEnded', 'sessionQueued', 'intentNotRecognized', 'startSession','continueSession', 'endSession'],
					'hermes/hotword/':         ['toggleOn', 'toggleOff', HOTWORD + '/detected'],
					'hermes/asr/':             ['toggleOn', 'toggleOff', 'startListening', 'stopListening', 'partialTextCaptured', 'textCaptured'],
					'hermes/nlu/':             ['query', 'partialQuery', 'intentParsed', 'slotParsed', 'intentNotRecognized'],
					'hermes/intent/':          ['#'],
					'hermes/error/':           ['nlu'],
					'hermes/tts/':             ['say', 'sayFinished']}


# Defne transitions
states=[State(name='Init',),
		State(name='WaitingRhasspySession',),
		State(name='SessionActive',),
		State(name='WaitingIntent',),
		State(name='WaitingSpeech',),
		State(name='WaitingSay',),]


transitions = [
    # Session start
	{ 'trigger': 'external_request', 'source': 'Init', 'dest': 'WaitingRhasspySession'}, # When an external client start a new interaction
	{ 'trigger': 'wake_up', 'source': ['Init', 'SessionActive', 'WaitingSpeech', 'WaitingIntent', 'WaitingRhasspySession'], 
	  'dest': 'WaitingRhasspySession', 'before': ['check_hotword','close_session','init_session']},  # During an action a wake up word is interpreted as an interrupt and a subsequent session close. A new session has to be intiated conseuqeuntly

	# rhasspy session
	{ 'trigger': 'session_started', 'source': 'WaitingRhasspySession', 'dest': 'SessionActive', 
	  'before': ['check_rhasspy_site_data','rhasspy_session_started']}, 
	{ 'trigger': 'asr_started', 'source': 'SessionActive', 'dest': 'WaitingSpeech', 'before': 'check_session_data'}, 
	{ 'trigger': 'text_captured', 'source': 'WaitingSpeech', 'dest': 'WaitingIntent', 'before': 'check_session_data'}, 
	
	# after check_rhasspy_site_data intent is (not) recognized. For now we go back in session active...
	{ 'trigger': 'intent_recognized', 'source': 'WaitingIntent', 'dest': 'SessionActive', 'before': 'check_session_data'}, 
	{ 'trigger': 'intent_not_recognized', 'source': 'WaitingIntent', 'dest': 'SessionActive', 'before': 'check_session_data'}, 
	
	# when check_rhasspy_site_data session is closed go back to a proper starting point
	{ 'trigger': 'rh_session_reset', 'source': ['WaitingRhasspySession'], 'dest': 'WaitingRhasspySession', 
	  'before': ['check_rhasspy_site_data','rhasspy_session_terminated_during_wait']}, # the exteranl session has been terminated
	{ 'trigger': 'rh_session_reset', 'source': ['SessionActive','WaitingSpeech','WaitingSay','WaitingIntent'], 
	  'dest': 'WaitingRhasspySession', 'before': ['check_session_data', 'rhasspy_session_terminated']}, # the exteranl session has been terminated
	
	# when session is closed go back to
	#{ 'trigger': 'wake_up', 'source': 'SessionActive', 'dest': 'WaitingRhasspySession'}, 	
]

# ------------
# --- CODE ---
# ------------ 

class DialogueException(Exception):
	pass


class DialogueSessionException(Exception):
	pass


class SessionInfo:
	
	session_ID = None  # The internal ID (it is open by an external client or associated with a user interrupt (e.g. via hotword)
	session_time_start = None
	actions_list = []
	active_rhasspy_session = None  # The RH dialogue session
	site_rhasspy_id = ''  # The RH site where the dialogue is performed
	active_rhasspy_session_time_start = None
	
	def terminate_session(self):
		self.session_ID = None
		self.session_time_start  = None # Annotate when the session was originally started
		self.actions_list = [] # the actions executed during this session
		# Information about the actual rhasspy session
		self.reset_rhasspy_session_and_site()
		
	def reset_rhasspy_session_and_site(self):#, **kwargs):
		self.reset_rhasspy_session()
		self.reset_rhasspy_site()

	def reset_rhasspy_site(self):
		self.site_rhasspy_id = ''

	def reset_rhasspy_session(self):
		self.active_rhasspy_session = None
		self.active_rhasspy_session_time_start  = None
		
	def init_session(self, session_ID, rh_site):
		"""
		Init a dialogue session with session_ID for the rhasspy site rh_site
		"""
		self.session_ID = session_ID
		self.site_rhasspy_id = rh_site
		self.session_time_start = current_milli_time()
	
	def update_rhasspy_session(self, rh_session, rh_site):
		assert self.session_ID is not None, "update_rhasspy_session: session_ID cannot be None"
		self.active_rhasspy_session = rh_session
		self.active_rhasspy_session_time_start  = current_milli_time()
	
	@property
	def elasped_time(self):
		return current_milli_time()-self.session_time_start if self.session_time_start is not None else 0.0 #if self.has_active_rhasspy_session() else None
	
	@property
	def rhasspy_elasped_time(self):
		return current_milli_time()-self.active_rhasspy_session_time_start if self.active_rhasspy_session_time_start is not None else 0.0 #if self.has_active_rhasspy_session() else None
			
	
class DialogueSession:
	# TODO, add a session storage here??

	def __init__(self):
		self._session_data = SessionInfo()
		self.anonymous_id = 0
	
	# ----------------------------
	# ----- before functions -----
	# ----------------------------
	def reset_rhasspy_session(self, **kwargs):
		payload = kwargs['payload']
		rospy.logdebug("\tCalling reset_rhasspy_session_and_site: {}".format(payload))
		self._session_data.reset_rhasspy_session_and_site()
	
	def close_session(self, **kwargs):
		# TODO, i should save the action list of this session
		payload = kwargs['payload']
		rospy.logdebug("\tCalling terminate_session: {}".format(payload))
		self._session_data.terminate_session()
		
	def rhasspy_session_started(self, **kwargs):
		payload = kwargs['payload']
		rospy.logdebug("\tCalling start_session payload: {}".format(payload)) 
				
		# update the data
		self._session_data.update_rhasspy_session(payload['sessionId'],  payload['siteId'])
	
	def rhasspy_session_terminated(self, **kwargs):
		payload = kwargs['payload']
		
		termination_reason =  payload['termination']['reason']
		if termination_reason == 'timeout' or termination_reason == 'intentNotRecognized':
			rospy.logdebug("Calling session_data.reset_rhasspy_session due to termination reason: ->{}<-".format(termination_reason))
			self._session_data.reset_rhasspy_session()
		else:
			print("Unmanaged reason  in rhasspy_session_terminated: {}".format(termination_reason), file=stderr)
			rospy.logwarn("Unmanaged reason  in rhasspy_session_terminated: {}".format(termination_reason))
	
	def rhasspy_session_terminated_during_wait(self, **kwargs):
		payload = kwargs['payload']
		termination_reason =  payload['termination']['reason']
		
		# Based on termination reason i should change accordingly the behaviour
		# if termination_reason == 'intentNotRecognized':
		#elif termination_reason == 'timeout':
		#elif termination_reason == 'nominal': 
		if termination_reason == 'abortedByUser':
			rospy.logdebug('Calling session_data.reset_rhasspy_session_and_site abortedByUser')
			self._session_data.reset_rhasspy_session()
		else:
			print("Unmanaged reason  in rhasspy_session_terminated_during_wait: {}".format(termination_reason), file=stderr)
			rospy.logwarn("Unmanaged reason  in rhasspy_session_terminated_during_wait: {}".format(termination_reason))
		
	def init_session(self, **kwargs):
		payload = kwargs['payload']
		rospy.logdebug("\tCalling init_session, payload: {}".format(payload))
		
		# TODO: Code valid only for wake up word!!
		# if wakeupword, e.g. 'specific_topic': 'snowboy/detected', 
		self.anonymous_id += 1
		session_ID = WAKEUPWORD_PREFIX + HOTWORD +'_' + str(self.anonymous_id)
		rh_site = payload['siteId']
		self._session_data.init_session(session_ID, rh_site)
	
	def check_hotword(self, **kwargs):
		payload = kwargs['payload']
		if not payload['modelId'] == HOTWORD:
			raise DialogueException('Wrong how word, got {} - expected {}'.format(payload['modelId'], HOTWORD))  
	
	def check_rhasspy_site_data(self, **kwargs):
		payload = kwargs['payload']
		rospy.logdebug("\tCalling check_rhasspy_site_data, payload: {}".format(payload))
		self._check_valid_rh_session(rh_site_id=payload['siteId'])
	
	def check_session_data(self, **kwargs):
		payload = kwargs['payload']
		rospy.logdebug("\tCalling check_session_data, payload: {}".format(payload))
		
		# TODO: move in the function _check_valid_rh_session (and rename it!).
		#  the session ID should be compared with the one extracted from session_data
		if self._session_data.session_ID==None:
			raise DialogueSessionException("On enter STATE SessionActive, session_ID[{}] cannot be None".format(
											self._session_data.session_ID ))
		self._check_valid_rh_session(rh_session_id=payload['sessionId'], rh_site_id=payload['siteId'])
		#if not (self._session_data.session_ID is not None and self._session_data.site_rhasspy_id!='' is not None and self._session_data.active_rhasspy_session is not None):
		
	# ---------------------
	# ----- Utilities -----
	# ---------------------
	def is_valid_rh_session(self, rh_session_id, rh_site_id):
		return self._session_data.active_rhasspy_session==rh_session_id and self._session_data.site_rhasspy_id == rh_site_id
		
	def get_session_ID(self, rh_session_id, rh_site_id):
		if self.is_valid_rh_session(rh_session_id, rh_site_id):
			return self._session_data.session_ID
		else:
			raise DialogueException('Cannot return session ID: request rh pair({}-{}) mismatch persited pair ({}-{})'.format(
									rh_session_id, site_id, self._session_data.active_rhasspy_session, self._session_data.site_rhasspy_id))
	
	# ------------------------------------
	# ----- Internal Check Utilities -----
	# ------------------------------------
	def _check_valid_rh_session(self, rh_session_id=None, rh_site_id=''):
		
		if self._session_data.active_rhasspy_session!=rh_session_id:
			raise DialogueSessionException(
					"On enter STATE SessionActive, requested session [{}] is not the active session [{}]".format(
					rh_session_id, self._session_data.active_rhasspy_session ))
		if self._session_data.site_rhasspy_id!=rh_site_id:
			raise DialogueSessionException(
					"On enter STATE SessionActive, requested site [{}] is not the active site [{}]".format(
					rh_site_id, self._session_data.site_rhasspy_id))
	
	# ----------------------
	# ----- Init STATE -----
	# ----------------------
	def on_enter_Init(self, **kwargs):
		rospy.loginfo("\t+-+-+-+-+-+-+-ENTER Init: from kwargs={}".format(kwargs))
		if not (self._session_data.session_ID is None and self._session_data.active_rhasspy_session is None):
			raise DialogueSessionException("On enter STATE Init, Session check failed: session_ID[{}] and active_rhasspy_session should be None: [{}]".format(
											self._session_data.session_ID, self._session_data.active_rhasspy_session))
		# todo I should reset the session?
	#	rospy.loginfo("DialogueSession: reset rhasspy session {} after {} ms".format(self._active_rhasspy_session, self.elasped_time))
	
	def on_exit_Init(self, **kwargs):
		rospy.loginfo("\t\t*/*/*/*/*/*/*/EXIT Init: kwargs={}".format(kwargs))
		

	# ---------------------------------------
	# ----- WaitingRhasspySession STATE -----
	# ---------------------------------------
	def on_enter_WaitingRhasspySession(self, **kwargs):
		rospy.loginfo("ENTER\t+-+-+-+-+-+-+- WaitingRhasspySession: kwargs={}".format(kwargs))
		# print(self._session_data.session_data_ID, self._session_data.active_rhasspy_session, self._session_data.site_rhasspy_id)
		# assert self._session_data.session_data_ID is not None \
				# and self._session_data.active_rhasspy_session is None \
				# and self._session_data.site_rhasspy_id is not None, "WaitingRhasspySession: Session not properly resetted"
		if not (self._session_data.session_ID is not None and self._session_data.site_rhasspy_id!='' and self._session_data.active_rhasspy_session is None):
			raise DialogueSessionException("On enter STATE WaitingRhasspySession, Session not properly resetted: session_ID[{}] "
										   "and active_rhasspy_session[{}] should be NOT None or NOT empty - active_rhasspy_session should be None [{}]".format(
											self._session_data.session_ID, self._session_data.site_rhasspy_id, self._session_data.active_rhasspy_session))
		
	def on_exit_WaitingRhasspySession(self, **kwargs):
		rospy.loginfo("EXIT\t*/*/*/*/*/*/*/EXIT DialogueSession.on_exit_WaitingRhasspySession: kwargs={}".format(kwargs))

	# -------------------------------
	# ----- SessionActive STATE -----
	# -------------------------------
	def on_enter_SessionActive(self, **kwargs):
		rospy.loginfo("ENTER\t+-+-+-+-+-+-+- SessionActive: kwargs={}".format(kwargs))						
	
	def on_exit_SessionActive(self, **kwargs):
		rospy.loginfo("EXIT\t*/*/*/*/*/*/*/ SessionActive: kwargs={}".format(kwargs))

	# -------------------------------
	# ----- WaitingIntent STATE -----
	# -------------------------------
	def on_enter_WaitingIntent(self, **kwargs):
		rospy.loginfo("ENTER\t*/*/*/*/*/*/*/ WaitingIntent: kwargs={}".format(kwargs))
	
	def on_exit_WaitingIntent(self, **kwargs):
		rospy.loginfo("EXIT\t*/*/*/*/*/*/*/ WaitingIntent: kwargs={}".format(kwargs))
		
	# -------------------------------
	# ----- WaitingSpeech STATE -----
	# -------------------------------
	def on_enter_WaitingSpeech(self, **kwargs):
		rospy.loginfo("ENTER\t*/*/*/*/*/*/*/ WaitingSpeech: kwargs={}".format(kwargs))
	
	def on_exit_WaitingSpeech(self, **kwargs):
		rospy.loginfo("EXIT\t*/*/*/*/*/*/*/EXIT WaitingSpeech: kwargs={}".format(kwargs))

	# ----------------------------
	# ----- WaitingSay STATE -----
	# ----------------------------
	def on_enter_WaitingSay(self, **kwargs):
		rospy.loginfo("ENTER\t*/*/*/*/*/*/*/ENTER WaitingSay: kwargs={}".format(kwargs))
	
	def on_exit_WaitingSay(self, **kwargs):
		rospy.loginfo("EXIT\t*/*/*/*/*/*/*/EXIT WaitingSay: kwargs={}".format(kwargs))


class DialogueManager(ManagerInterface):
	"""
	DialogueManager is in charge of managing all the external interaction with ROS service and topics, in it is embedded
	the State machine of the Dialogue.
	A dialogue can arrive from outside requesting an interaction, with or without a custom contex_id
	"""

	# _prog_number = 1
	# _session_dict = FixSizeOrderedDict(max=3)
	_USE_INTENT_SERVICE = False # a kind of backward compatibility... should be removed

	def __init__(self, mqtt_client, args):
		super().__init__(mqtt_client, args)
		self._USE_INTENT_SERVICE = args.call_intent_service
		self._session = DialogueSession()
		self._dialogue_sm = Machine(model=self._session, states=states, transitions=transitions, initial=INIT_STATE)

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

	# ---------------------------------
	# -------------- ROS --------------
	# ---------------------------------
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
			response = self._perform_ros_interaction_srv(request, utter_only=True)
			rospy.logdebug("executed utter only with response: {}".format(response))
		except Exception as e:
			rospy.logdebug('Catched excpetion exceuting uttering: {}'.format(e))
			response = InteractServiceResponse()
			response.success = False
		return response
	
	def execute_interaction_ros_service(self, request):
		try:
			response = self._perform_ros_interaction_srv(request, utter_only=False)
			rospy.logdebug("executed inteaction with response: {}".format(response))
		except Exception as e:
			rospy.logdebug('Catched excpetion exceuting interaction: {}'.format(e))
			response = InteractServiceResponse()
			response.success = False
		return response

	def _perform_ros_interaction_srv(self, request, utter_only=False):
		# unique method to manage a request for a ros service
		response = InteractServiceResponse()
		context_id = request.context_id

		if utter_only:
			interaction_type='notification'
		else:
			interaction_type='action'
		if context_id is not None and len(context_id):
			rospy.logdebug("Recieved ROS request for interaction {} with context_id {}".format(interaction_type, context_id))
			retval = self.direct_interaction(type=interaction_type, text=request.text, canBeEnqueued=request.canbeenqued,
											 canBeDiscarded=request.canbediscarded ,customData=context_id)
			#print("execute_interaction_ros_service with context retval=", retval)
		else:
			rospy.logdebug("Recieved ROS request for interaction {} with anonymous context".format(interaction_type))
			retval = self.direct_interaction(type=interaction_type, text=request.text, canBeEnqueued=request.canbeenqued,
											 canBeDiscarded=request.canbediscarded )
			#print("execute_interaction_ros_service NO context retval=", retval)
		# response.success = retval # If only one argument bool is returned in the .srv file i received a
		# Error processing request: field success is not a bool
		response = retval
		# rospy.logdebug("Return ROS request with response: {}".format(response))
		return response

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
	# def execute_new_dialogue_ros_service(self, request):
		# pass
	# def execute_end_dialogue_ros_service(self, request):
		# pass

	def publish_not_recognized_intent(self, payload):
		input_str = payload["input"]
		site_id = payload["siteId"]
		session_id = payload['sessionId']
		custom_data = payload['customData']
		context_id = ''  
		# todo check if is fine
		assert self._session.is_valid_rh_session(rh_session_id,site_id), "Mismatched rh session"
		context_id = self._session.get_session_ID(rh_session_id, site_id) # to check if is fine

		# send ros massage
		msg = IntentNotRecognizedMessage(context_id=context_id, original_input=input_str)
		rospy.logdebug("received payload {} and publishing message: {} ".format(payload, msg))
		self.pub_intent_not_rec.publish(msg)

	# --------------------------------------------------
	# -------------- HIGH LEVEL FUNCTIONS --------------
	# --------------------------------------------------
	
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
			rospy.loginfo("Previous session running {},  MQTT topic can be enqued {}".format(self._session.get_session_info(), topic))
			_publish()
			retval = True
		elif self._session.has_active_rhasspy_session():
			if canBeDiscarded:
				rospy.loginfo("Previous session running {}, discard publish MQTT topic {}".format(self._session.get_session_info(), topic))
				retval = False
			else:
				# sessionId:	String - Identifier of the session to end.
				# text :	 	Optional String - The text the TTS should say to end the session.
				session_id = self._session.rhasspy_session
				payload = json.dumps({"sessionId": session_id})
				rospy.loginfo("Previous session running {}, removed, publish topic {}".format(self._session.get_session_info(), topic))
				self._mqtt_client.publish(topic="hermes/dialogueManager/endSession", payload=payload)
				print('wait', end="")
				while not  self._session.has_active_rhasspy_session():
					print('.', end="")
					rospy.sleep(0.001) # better sleep before publish?
				print()
				_publish()
				retval = True
		else:
			rospy.loginfo("Previous session running {},  MQTT topic is enqued {}".format(self._session.get_session_info(), topic))
			_publish()
			retval = True
		
		return retval


	# -------------------------------------------
	# -------------- MQTT HANDLERS --------------
	# -------------------------------------------
	
	# THIS IS NOT AN HANDLER BUT AN UTILITY FUNCTION
	# Where do i self.pub_intent_not_rec.publish(msg) 
	
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
		rh_session_id = payload["sessionId"]
		assert self._session.is_valid_rh_session(rh_session_id,site_id), "Mismatched rh session"
		context_id = self._session.get_session_ID(rh_session_id, site_id) # to check if is fine
		
		# TODO,check of the session has to be done in the proper method?
		#if self._session.is_valid_rh_session(rh_session_id=payload['sessionId'], rh_site_id=payload['siteId']):
		#	context_id = self._session.session
		# TODO: add the context!! This is 
		
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
		# rospy.logdebug("->{}<- input, intent ->{}<- is coded as ROS message: {} - payload: {} - confidence: {} ".format(input_str, intent_str, ros_intent_name, ros_pay_load, ros_confidence))
		
		try:
			self._session.intent_recognized(payload=payload)
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
		#_print_status('handle_dialogue', self._session, enter=True)
		rospy.logdebug("+-+-+-+-+-+-Enter with payload={}".format(payload))
		if specific_topic == 'sessionStarted':
			# reset the timer
			time_ref  = current_milli_time()
			# _print_topic_data('handle_dialogue', specific_topic, payload, parameters)
			# two cases, I have started the session OR the session was an interrupt from the user
			# a new session, I want to register it
			rospy.logdebug("+-+-+-+-+-+-Calling start_session: payload={}".format(payload))
			try:
				# self._session.start_session(rh_session_id=payload['sessionId'], rh_site_id=payload['siteId'], session_data=payload['customData'] )
				self._session.session_started(specific_topic=specific_topic, payload=payload) #rh_session_id=payload['sessionId'], rh_site_id=payload['siteId'], session_data=payload['customData'] )
			except MachineError as e:
				rospy.logwarn('+-+-+-+-+-+-ERROR: '+str(specific_topic)+':  %s', str(e))
			except Exception as e:
				rospy.logerr('+-+-+-+-+-+-EXCEPTION: '+str(specific_topic)+':  %s', str(e))
				raise e
		elif specific_topic == 'sessionQueued':
			_print_topic_data('handle_dialogue', specific_topic, payload, parameters)
		elif specific_topic == 'sessionEnded':
			#_print_topic_data('handle_dialogue', specific_topic, payload, parameters + ['termination'])
			try:
				rospy.logdebug("+-+-+-+-+-+-Calling rh_session_reset termination={}: payload={}".format( payload['termination'], payload))
				self._session.rh_session_reset(specific_topic=specific_topic, payload=payload)
			except MachineError as e:
				rospy.logwarn('+-+-+-+-+-+-ERROR:'+str(specific_topic)+':  %s', str(e))
			except Exception as e:
				rospy.logerr('+-+-+-+-+-+-EXCEPTION:'+str(specific_topic)+':  %s', str(e))
				raise e
		elif specific_topic == 'startSession':
			rospy.loginfo('handle_dialogue: startSession {}-{}'.format(specific_topic, payload))
		elif specific_topic == 'continueSession':
			rospy.loginfo('handle_dialogue: continueSession {}-{}'.format(specific_topic, payload))
		else:
			rospy.logwarn('this is the handle_dialogue, not suppoorted topic->' + str(specific_topic))
		#_print_status('handle_dialogue', self._session, enter=False)

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
		elif specific_topic == HOTWORD + '/detected':
			#_print_topic_data('handle_hotword->', specific_topic, payload, None)
			self._session.wake_up(specific_topic=specific_topic, payload=payload)
		else:
			rospy.logwarn('handle_hotword->{} +++UNHANDLED+++'.format(specific_topic))

	def handle_asr(self, specific_topic, client, userdata, message):
		payload = json.loads(message.payload)
		if specific_topic == 'startListening':
			rospy.loginfo('==========handle_asr-startListening: '+str(specific_topic) +': payload %s', str(payload))
			self._session.asr_started(specific_topic=specific_topic, payload=payload)
			#_print_topic_data('handle_asr-startListening', specific_topic, payload, None)
		elif specific_topic == 'stopListening':
			rospy.loginfo('++++++++++handle_asr-stopListening: '+str(specific_topic) +': payload %s', str(payload))
			#_print_topic_data('handle_asr-stopListening', specific_topic, payload, None)
		elif specific_topic == 'textCaptured':
			rospy.loginfo('**********handle_asr-textCaptured: '+str(specific_topic) +': payload %s', str(payload))
			self._session.text_captured(specific_topic=specific_topic, payload=payload)
		else:
			pass
			#_print_topic_data('handle_asr-other: ', specific_topic, payload, None)

	def handle_nlu(self, specific_topic, client, userdata, message):
		payload = json.loads(message.payload)
		parameters = ['sessionId', 'siteId', 'input']
		#_print_status('handle_nlu', self._session, enter=True)
		if specific_topic == 'intentParsed':
			# Note: every single intent is recorded and a message automatically sent
			rospy.logdebug('handle_nlu.intentParsed {} - payload={}'.format(specific_topic, payload,))
			# accordingly to https://docs.snips.ai/reference/hermes#obtaining-the-result-of-an-nlu-parsing-low-level-api
			# Note that this is a low-level API, and it is not recommended to be used of production. 
			# In particular, this API method does not guarantee that all slots of the intent have been properly parsed. 
			# To detect an intent parsed by the NLU component, it is recommended to subscribe to the following topic instead:
			# hermes/intent/<INTENT_NAME>
			# The state change is called within handle_intents, whenever a specific intent is recognized
		elif specific_topic == 'intentNotRecognized':
			try:
				# when an intent is not recognized the mechanism for parsed intent is not in place
				# this has to be called manually
				rospy.logdebug('handle_nlu.intentNotRecognized {} - payload={}'.format(specific_topic, payload))
				# call the state machine
				self._session.intent_not_recognized(specific_topic=specific_topic, payload=payload)
				# publish ros topic 
				self.publish_not_recognized_intent(payload)
			except MachineError as e:
				rospy.logwarn('+-+-+-+-+-+-ERROR:'+str(specific_topic)+':  %s', str(e))
			except Exception as e:
				rospy.logerr('+-+-+-+-+-+-EXCEPTION:'+str(specific_topic)+':  %s', str(e))
				raise e
		else:
			pass #rospy.logwarn('handle_nlu not managed {}-{}'.format(specific_topic, payload))
		#_print_status('handle_nlu', self._session, enter=False)

	def handle_error(self, specific_topic, client, userdata, message):
		print('this is the handle_error->',specific_topic)

	def handle_tts(self, specific_topic, client, userdata, message):
		payload = json.loads(message.payload)
		rospy.logdebug("handling {} - payload={} ".format(specific_topic, payload))
		session_data = self._session.get_session_from_rh_session(rh_session_id=payload['sessionId'], rh_site_id=payload['siteId'])
		text_uttered = '' if not 'text' in  payload else  payload['text']
		msg = TtsSessionEnded(context_id=session_data, text_uttered=text_uttered)
		rospy.logdebug("publishing ROS message pub_tts_finished: {} ".format(msg))
		self.pub_tts_finished.publish(msg)
		
		
# class DialogueSession_OLD:
	# """
	# A model for the State Machine in transitions
	
	# """

	# _active_rhasspy_session = None
	# _site_rhasspy_id = ''
	# time_start = None
	# session_data = None
	# actions_list = []
	# _anonymous_id = 0

	# def __init__(self):
		# self.reset()

	# def reset(self, **kwargs):
		# rospy.logdebug("DialogueSession: reset session={}, rh_session_id={} after {} ms".format(self.session_data, self._active_rhasspy_session, self.elasped_time))
		# self._active_rhasspy_session = None
		# self._time_start  = None
		# self._site_rhasspy_id = None
		# self._session_data = None
		# self.actions_list = []

	# def reset_rhasspy_session_and_site(self, **kwargs):
		# rospy.logdebug("DialogueSession: reset rhasspy session {} after {} ms".format(self._active_rhasspy_session, self.elasped_time))
		# self._active_rhasspy_session = None
		# self._site_rhasspy_id = None

	# def on_enter_WaitingIntent(self, rh_session_id=None, rh_site_id=None, session_data=None, action=None):#,rh_session_id = None, rh_site_id = None, session_data = None):
		# pass #rospy.logdebug("on_enter_WaitingIntent: rh_session_id={} rh_site_id={} session_data={}".format(self._active_rhasspy_session, self._site_rhasspy_id, self._session_data))

	# def on_exit_WaitingIntent(self, rh_session_id=None, rh_site_id=None, session_data=None, action=None):
		# # register any detected action
		# pass#rospy.logdebug("on_exit_WaitingIntent: rh_session_id={} rh_site_id={} session_data={}".format(self._active_rhasspy_session, self._site_rhasspy_id, self._session_data))

	# def on_enter_WaitingSession(self, **kwargs):
		# # rospy.logdebug("on_enter_WaitingSession: rh_session_id={} rh_site_id={} session_data={}".format(self._active_rhasspy_session, self._site_rhasspy_id, self._session_data))

		# if self._active_rhasspy_session is not None:
			# pass#rospy.logdebug("on_exit_WaitingSession: rh_session_id={} rh_site_id={} session_data={}".format(rh_session_id, rh_site_id, session_data))

	# def on_exit_WaitingSession(self,rh_session_id, rh_site_id, session_data = None):
		# # Call exiting from the status, this init the new session
		# #rospy.logdebug("on_exit_WaitingSession: rh_session_id={} rh_site_id={} session_data={}".format(rh_session_id, rh_site_id, session_data))

		# if self.has_active_rhasspy_session():
			# raise DialogueException('Rhasspy session cannot be none')

		# if self.has_session_data():
			# # Follow cases
			# if session_data is None:
				# # i have a valid session but an anonymous session is requested, there should be a priority??
				# # rospy.logdebug("on_exit_WaitingSession: request an anoymous session with existing session data: %s", self.session)
				# # archive session, actions etc
				# rospy.logdebug("on_exit_WaitingSession: call reset with session data: %s", session_data)
				# self.reset() # is it safe? i wanna clear the status
			# elif session_data == self.session:
				# rospy.logdebug("on_exit_WaitingSession: an existing external session to continue: %s", session_data)
				# # update only the
				# self._active_rhasspy_session = rh_session_id
				# self._site_rhasspy_id = rh_site_id
				# return # is it safe? i wanna clear the status
			# else:
				# rospy.logdebug("on_exit_WaitingSession: a new external session to continue: %s", session_data)
				# self.reset()

		# # initiate the session
		# if session_data is None:
			# self._anonymous_id += 1
			# session_data = ANONYMOUS_PREFIX + str(self._anonymous_id)
			# rospy.logdebug("on_exit_WaitingSession: starting anonymous session: {}".format(session_data))

		# rospy.logdebug("DialogueSession: starting new dialogue={} rh_sess_id={}, rh_site_id={} at {} ms".format(session_data, rh_session_id, rh_site_id, current_milli_time() ))
		# # assumption, no previous session are running...
		# assert self._active_rhasspy_session is None and self._session_data is None and len(self.actions_list)==0, "Cannot create new session if previous was not reset"
		# self._active_rhasspy_session = rh_session_id
		# self._site_rhasspy_id = rh_site_id
		# self._session_data = session_data
		# self._time_start  = current_milli_time()

	# def has_active_rhasspy_session(self):
		# return not self.rhasspy_session == None

	# def is_active_rhasspy_session(self, rh_session_id):
		# return self.rhasspy_session == rh_session_id

	# def is_valid_rh_session(self, rh_session_id, rh_site_id):
		# return self.is_active_rhasspy_session(rh_session_id=rh_session_id) and self._site_rhasspy_id == rh_site_id

	# def is_session_data(self, session_data):
		# return self._session_data == session_data

	# def has_session_data(self):
		# return not self._session_data == None

	# def get_session_from_rh_session(self, rh_session_id, rh_site_id):
		# if self.is_valid_rh_session(rh_session_id, rh_site_id):
			# return self.session
		# else:
			# return None

	# @property
	# def actions(self):
		# return self.actions

	# @property
	# def rhasspy_session(self):
		# return self._active_rhasspy_session

	# @rhasspy_session.setter
	# def rhasspy_session(self, new_rhasspy_session_id):
		# self._active_rhasspy_session  = new_rhasspy_session_id

	# @property
	# def rhasspy_site(self):
		# return self._site_rhasspy_id if self.has_active_rhasspy_session() else None

	# @property
	# def session(self):
		# return self._session_data # if self.has_active_session() else None

	# @property
	# def site(self):
		# return self._site_rhasspy_id if self.has_active_rhasspy_session() else None

	# @property
	# def elasped_time(self):
		# return current_milli_time()-self._time_start if self.has_active_rhasspy_session() else None

	# @property
	# def start_time(self):
		# return self._time_start if self.has_active_rhasspy_session() else None

	# def add_action(self, rh_session_id, rh_site_id, session_data, action):
		# assert isinstance(action, dict) and 'result' in action, 'Action wrong format or result key not available'

		# if self.get_session_from_rh_session(rh_session_id, rh_site_id) == session_data:
			# rospy.loginfo("add_action:  adding action={},".format(action ))
			# self.actions_list.append((current_milli_time(), action))
			# return True
		# else:
			# rospy.logwarn("add_action: doing nothing incompatible request(session={} or site{} or customData={}) actual session (session={}, rh_site_id={}, customData={})".format(
							# rh_session_id,
							# rh_site_id,
							# custom_data,
							# self._active_rhasspy_session,
							# self._site_rhasspy_id,
							# self.session))
			# return False

	# # TODO:next methods needs to reviewed.
	# def get_session_info(self):
		# """
		# If has an active session returns the rhasspy session id as defined in https://docs.snips.ai/reference/dialogue#session-started
		# It records also start time etc.
		# """
		# if self.has_active_rhasspy_session():
			# return self._active_rhasspy_session, self._session_data, self._site_rhasspy_id, self._time_start, current_milli_time()-self._time_start
		# else:
			# return None