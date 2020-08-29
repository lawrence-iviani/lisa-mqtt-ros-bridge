
from collections import OrderedDict
import json

# from rhasspyhermes.asr import (
    # AsrStartListening,
    # AsrStopListening,
    # AsrTextCaptured,
    # AsrToggleOff,
    # AsrToggleOn,
    # AsrToggleReason,
# )
# from rhasspyhermes.audioserver import AudioPlayBytes, AudioPlayFinished
# from rhasspyhermes.base import Message
# from rhasspyhermes.client import GeneratorType, HermesClient, TopicArgs
#from rhasspyhermes.dialogue import (
    # DialogueAction,
#    DialogueActionType,
    # DialogueConfigure,
    # DialogueContinueSession,
    # DialogueEndSession,
    # DialogueError,
    # DialogueIntentNotRecognized,
    # DialogueNotification,
    # DialogueSessionEnded,
    # DialogueSessionQueued,
    # DialogueSessionStarted,
    # DialogueSessionTermination,
    # DialogueSessionTerminationReason,
    # DialogueStartSession,
#)
# from rhasspyhermes.nlu import NluIntent, NluIntentNotRecognized, NluQuery
# from rhasspyhermes.tts import TtsSay, TtsSayFinished
# from rhasspyhermes.wake import (
    # HotwordDetected,
    # HotwordToggleOff,
    # HotwordToggleOn,
    # HotwordToggleReason,
# )

import time
current_milli_time = lambda: int(round(time.time() * 1000))
time_ref  = current_milli_time()

def _print_debug(msg):
	print("[{}ms]-{}".format(current_milli_time()-time_ref, msg))

# https://stackoverflow.com/questions/49274177/need-python-dictionary-to-act-like-deque-have-maximum-length
class FixSizeOrderedDict(OrderedDict):
	def __init__(self, *args, max=0, **kwargs):
		self._max = max
		super().__init__(*args, **kwargs)

	def __setitem__(self, key, value):
		OrderedDict.__setitem__(self, key, value)
		if self._max > 0:
			if len(self) > self._max:
				self.popitem(False)


class ExternalContextSession:
	MAX_CONTEXT_LENGTH = 10  # the max length of action/interaction session (this the mimimum element??)

	# possible states
	INVALID = -1
	ACTIVE=0
	SUSPENDED=1
	UTTERING=2

	_list_rh_sessions = None

	def __init__(self, client_id):
		self.client_id=client_id
		self._list_rh_sessions=FixSizeOrderedDict(max=self.MAX_CONTEXT_LENGTH) 
		self.state=self.INVALID

	def add_rhasspy_session(rhasspy_session):
		if rhasspy_session is None:
			return False
		self._list_rh_sessions[rhasspy_session.session] = rhasspy_session
		return True
		

# here to add the handler information e.g self.handle_asr self.handle_dialogue etc
subscribe_topics = {'hermes/dialogueManager/': ['sessionStarted', 'sessionEnded', 'sessionQueued', 'intentNotRecognized', 'startSession','continueSession', 'endSession'],
					'hermes/hotword/':         ['toggleOn', 'toggleOff', 'default/detected'],
					'hermes/asr/':             ['toggleOn', 'toggleOff', 'startListening', 'stopListening', 'partialTextCaptured', 'textCaptured'],
					'hermes/nlu/':             ['query', 'partialQuery', 'intentParsed', 'slotParsed', 'intentNotRecognized'],
					'hermes/error/':           ['nlu'],
					'hermes/tts/':             ['say', 'sayFinished']}


class RhasspySession:

	active_rhasspy_session = None
	site_rhasspy_id = ''
	time_start = None
	
	def __init__(self):
		self.reset()
	
	def reset(self):
		_print_debug("RhasspySession: reset session {} after {} ms".format(self.active_rhasspy_session, self.elasped_time))
		self.active_rhasspy_session = None
		self.time_start  = None
		self.site_rhasspy_id = None
		
	def start(self, session_id, site_id ):
		self.active_rhasspy_session = session_id
		self.site_rhasspy_id = site_id
		self.time_start  = current_milli_time()
		_print_debug("RhasspySession: starting session={}, site_id={} at {} ms".format(self.active_rhasspy_session, self.site_rhasspy_id, self.time_start ))
		
	def has_active_session(self):
		return not self.active_rhasspy_session == None
	
	def get_session_info():
		"""
		If has an active session returns the rhasspy session id as defined in https://docs.snips.ai/reference/dialogue#session-started
		It records also start time etc.
		"""
		if has_active_session:
			return self.active_rhasspy_session, self.site_rhasspy_id, self.time_start, current_milli_time()-self.time_start
		else:
			return None
	
	@property
	def session(self):
		return self.active_rhasspy_session if self.has_active_session() else None
		
	@property
	def site(self):
		return self.site_rhasspy_id if self.has_active_session() else None
	
	@property
	def elasped_time(self):
		print()
		return current_milli_time()-self.time_start if self.has_active_session() else None
		
	@property
	def start_time(self):
		return self.time_start if self.has_active_session() else None
		

class DialogueManager:

	_prog_number = 1
	_active_rhasspy_session = RhasspySession()

	def __init__(self):
		self._session_dict = FixSizeOrderedDict(max=3)
		
	# this should become a start section
	def _crazy_send(self, client):
	
		# siteId Optional String - Site where to start the session
		# init Object - Session initialization description: Action or Notification. See below
		# customData Optional String - Additional information that can be provided by the handler. 
		# 	Each message related to the new session - sent by the Dialogue Manager - will contain this data
	
		# init Object 
		# can_be_enqueued: bool  If true, the session will start when there is no pending one on this site. Otherwise, the session is just dropped if there is running one.
		# type: DialogueActionType = ACTION
		# text: typing.Optional[str] = None   Text that the TTS should say at the beginning of the session.
		# intent_filter: typing.Optional[typing.List[str]] = None  A list of intents names to restrict the NLU resolution on the first query.
		# send_intent_not_recognized: bool = False
        # Indicates whether the dialogue manager should handle non-recognized intents by itself or send them for the client to handle.
		# init_pl = {"type": 'notification', "text": 'something to say', "canBeEnqueued": True} 
		init_pl = {"type": 'action', "text": 'something to infer', "canBeEnqueued": True} 
		_print_debug("_crazy_send: {}".format(init_pl))
		payload = json.dumps({"init": init_pl})  # TtsSay(text=sentence, lang="en_EN")  # id=sentence for example?
		_print_debug("_crazy_send: {}".format(payload))
		retval = client.publish(topic="hermes/dialogueManager/startSession", payload=payload)
		
	# callback
	def on_connect(self, client, userdata, flags, rc):
		_print_debug('+-+-+-+-dial_man.on_connect: {}'.format(*subscribe_topics))
		try:
			for k,v in subscribe_topics.items():
				client.subscribe(k+'#')
		except Exception as e:
			_print_debug('*********dial_man.on_connect with error: {}'.format(e))
		self._crazy_send(client)
		_print_debug('+-+-+-+-dial_man.on_connect FINISHED')
		
	# callback
	def on_message(self, client, userdata, message):
		try:
			# print('DialogueManager->DialogueStartSession')
			for k,v in subscribe_topics.items():
				if message.topic.startswith(k):
					specific_topic = message.topic[len(k):]
					# print('\nDialogueManager->', k,' - ', specific_topic)
					if 'dialogueManager' in k:
						self.handle_dialogue(specific_topic, message)
					elif 'hotword' in k:
						self.handle_hotword(specific_topic, message)
					elif 'asr' in k:
						self.handle_asr(specific_topic, message)
					elif 'nlu' in k:
						self.handle_nlu(specific_topic, message)
					elif 'error' in k:
						self.handle_error(specific_topic, message)
					elif 'tts' in k:
						self.handle_tts(specific_topic, message)
					else:
						continue
					break
		except Exception as e:
			_print_debug("error handling message {} -> {} - Payload was: {}".format(message.topic, e, message.payload))
		
	@staticmethod
	def _print_topic_data(handle_name, specific_topic, payload, parameters):
		_topic_msg = "\n\tparameters: " + "|".join(["{}={}".format(t, payload[t]) for t in parameters]) if parameters is not None else ""
		# if parameters is not None:
			# _topic_msg = "\n\tparameters: " + "|".join(["{}={}".format(t, payload[t]) for t in parameters])
		# else:
			# _topic_msg=""
		# print(_topic_msg)
		_print_debug('{}->{} payload:{} {}'.format(handle_name, specific_topic, payload, _topic_msg))
																						#payload['sessionId'], payload['siteId'], 
																						#payload['customData']))
		
	def handle_dialogue(self, specific_topic, message):
		payload = json.loads(message.payload)
		parameters = ['sessionId', 'siteId', 'customData']
		if specific_topic == 'sessionStarted':
			# reset the timer
			time_ref  = current_milli_time()
			self._print_topic_data('handle_dialogue', specific_topic, payload, parameters)
			# two cases, I have started the session OR the session was an interrupt from the user
			# a new session, I want to register it
			self._active_rhasspy_session.start(payload['sessionId'], payload['siteId'])
			# if payload['customData'] is not None:
				# add this session to 
				# payload['customData']

		elif specific_topic == 'sessionQueued': 
			self._print_topic_data('handle_dialogue', specific_topic, payload, parameters)
		elif specific_topic == 'sessionEnded':
			self._print_topic_data('handle_dialogue', specific_topic, payload, parameters + ['termination'])
			if self._active_rhasspy_session.session == payload['sessionId']:
				self._active_rhasspy_session.reset() #(payload['sessionId'], payload['siteId'])
			else:
				_print_debug("Ending a different session {} != {}, doing nothing...".format(self._active_rhasspy_session.session, payload['sessionId']))
		elif specific_topic == 'startSession':
			self._print_topic_data('handle_dialogue', specific_topic, payload, None)
			# take the custom data for check...
			#self.start_new_session
		else:
			print('this is the handle_dialogue, not suppoorted topic->',specific_topic)
	
	def handle_hotword(self, specific_topic, message):
		payload = json.loads(message.payload)
		if specific_topic == 'toggleOn':
			self._print_topic_data('handle_hotword', specific_topic, payload, ['siteId'])
			#print('toggleOn', payload)
			#_print_debug('handle_hotword->{} siteId={} '.format(specific_topic, payload['siteId']))
		elif specific_topic == 'toggleOff':
			self._print_topic_data('handle_hotword', specific_topic, payload, ['siteId'])
			#print('toggleOff', payload)
			#_print_debug('handle_hotword->{} siteId={}'.format(specific_topic, payload['siteId']))
		else:
			print('toggleOff', payload)
			_print_debug('handle_hotword->{} +++UNHANDLED+++'.format(specific_topic))
		
	def handle_asr(self, specific_topic, message):
		payload = json.loads(message.payload)
		if specific_topic == 'startListening':
			self._print_topic_data('handle_asr', specific_topic, payload, None)
		elif specific_topic == 'stopListening':
			self._print_topic_data('handle_asr', specific_topic, payload, None)
		else:
			self._print_topic_data('handle_asr', specific_topic, payload, None)
			
		
	def handle_nlu(self, specific_topic, message):
		pass#print('this is the handle_nlu->',specific_topic)

	def handle_error(self, specific_topic, message):
		pass#print('this is the handle_error->',specific_topic)
		
	def handle_tts(self, specific_topic, message):
		pass#print('this is the handle_tts->',specific_topic)
		
		
	def add_actual_session(self, client_id=None,):
		# when a new session is triggered
		client_id = 'anonymous' if client_id is None else client_id
	
	def start_new_session(self,  session_type='notification', text_to_utter=''):
		# I want to create a brand new session by sending the command 
		
		
		
		# Create the new session
		# siteId Optional String - Site where to start the session
		# init Object - Session initialization description: Action or Notification. See below
		# customData Optional String - Additional information that can be provided by the handler. 
		# 	Each message related to the new session - sent by the Dialogue Manager - will contain this data
	
		# init Object 
		# can_be_enqueued: bool  If true, the session will start when there is no pending one on this site. Otherwise, the session is just dropped if there is running one.
		# type: DialogueActionType = ACTION
		# text: typing.Optional[str] = None   Text that the TTS should say at the beginning of the session.
		# intent_filter: typing.Optional[typing.List[str]] = None  A list of intents names to restrict the NLU resolution on the first query.
		# send_intent_not_recognized: bool = False
        # Indicates whether the dialogue manager should handle non-recognized intents by itself or send them for the client to handle.
		# init_pl = {"type": 'notification', "text": 'something to say', "canBeEnqueued": True} 
		
		if session_type=='notification':
			init_pl = {"type": 'notification', "text": text_to_utter, "canBeEnqueued": True} 
		if session_type=='action':
			init_pl = {"type": 'action', "text": text_to_utter, "canBeEnqueued": True} 
		# _print_debug("start_new_session: {}".format(init_pl))
		payload = json.dumps({"init": init_pl, "customData": client_id})  # TtsSay(text=sentence, lang="en_EN")  # id=sentence for example?
		_print_debug("start_new_session: {}".format(payload))
		retval = client.publish(topic="hermes/dialogueManager/startSession", payload=payload)
		
		if session_id in  self._session_dict.keys():
			return False
		self._session_dict[client_id] = SessionState(client_id)
		return True
		
	def stop_actual_session(self, session_id):
		if session_id in  self._session_dict.keys():
			del self._session_dict[session_id]
			return True
		return False
		
	def is_session_active(self, session_id):
		return list(self._session_dict.keys())[0] == session_id
		
	def utter(self, session_id=None, message=''):
		pass
		
	def wait_for_intent(self, session_id=None, message=''):
		pass
