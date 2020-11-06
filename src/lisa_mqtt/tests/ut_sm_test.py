#!/usr/bin/env python3

import unittest
from collections import namedtuple

from lisa_mqtt.dialogue import DialogueSession, Machine, MachineError, states, transitions, INIT_STATE, DialogueSessionException, DialogueException

# Used as test data in sequence
TestData = namedtuple('TestData', ['transition', 'next_state', 'msg', 'non_possible_states'])


# ---------------------
# --- CONFIGURATION ---
# --------------------- 

# Here are described which transitions are NON possible from a specific state. Calling a transition in that state would cause an error
#ALL STATES				= 	    ['wake_up', 'external_request', 'session_started', 'asr_started', 'text_captured','intent_recognized','intent_not_recognized','rh_session_reset' ]
FROM_INIT_NON_POSSIBLE =	 	[          						'session_started', 'asr_started', 'text_captured', 'intent_recognized', 'intent_not_recognized','rh_session_reset' ]
FROM_WAIT_RH_NON_POSSIBLE  =	[          						   				   'asr_started', 'text_captured', 'intent_recognized', 'intent_not_recognized' ]
FROM_SESS_ACT_NON_POSSIBLE =	[          						'session_started', 				  'text_captured', 'intent_recognized', 'intent_not_recognized' ]
FROM_WAIT_SPCH_NON_POSSIBLE	=	[          						'session_started', 'asr_started',                  'intent_recognized', 'intent_not_recognized' ]
FROM_WAIT_INTENT_NON_POSSIBLE = [          						'session_started', 'asr_started', 'text_captured',]
FROM_WAIT_SAY_NON_POSSIBLE    =	[								'session_started', 'asr_started', 'text_captured','intent_recognized','intent_not_recognized', ]


# ---------------------
# --- TEST SEQUENCE ---
# --------------------- 

# Test sequence definition, used in the respective test
# The test are performed as sequence in the respective test

################################################
## DEFINITION FOR TestLisaStateMachineBaseClass#
################################################
# Example of messages from a real session
WAKE_UP_MSG_1 = {'modelId': 'snowboy', 'modelVersion': '', 'modelType': 'personal', 'currentSensitivity': 0.8, 'siteId': 'default', 'sessionId': None, 'sendAudioCaptured': None, 'lang': None,} # 'termination': {'reason': 'intentNotRecognized'}}
SESS_STARTED_MSG_1 = {'sessionId': 'default-snowboy-52460f67-54eb-45cd-b619-748cf8453b0d', 'siteId': 'default', 'customData': 'snowboy', 'lang': None}
START_LISTENING_MSG_1 = {'siteId': 'default', 'sessionId': 'default-snowboy-52460f67-54eb-45cd-b619-748cf8453b0d', 'lang': None, 'stopOnSilence': True, 'sendAudioCaptured': True, 'wakewordId': 'snowboy', 'intentFilter': None}
TEXT_CAPTURED_MSG_1 = {'text': 'what time is it', 'likelihood': 1, 'seconds': 1.3038434779991803, 'siteId': 'default', 'sessionId': 'default-snowboy-52460f67-54eb-45cd-b619-748cf8453b0d', 'wakewordId': None, 'asrTokens': None, 'lang': None}
INTENT_RECOGN_MSG_1 = {'input': 'what time is it', 'intent': {'intentName': 'GetTime', 'confidenceScore': 1.0}, 'siteId': 'default', 'id': None, 'slots': [], 'sessionId': 'default-snowboy-52460f67-54eb-45cd-b619-748cf8453b0d'}
INTENT_NOT_RECOGN_MSG_1 = {'input': 'set blue', 'siteId': 'default', 'id': None, 'customData': None, 'sessionId': 'default-snowboy-52460f67-54eb-45cd-b619-748cf8453b0d'}
RH_RESET_SESS_MSG_1 = {'termination': {'reason': 'timeout'}, 'sessionId': 'default-snowboy-52460f67-54eb-45cd-b619-748cf8453b0d', 'siteId': 'default', 'customData': 'snowboy'}
RH_RESET_SESS_DURING_WAIT_MSG_1 = {'termination': {'reason': 'abortedByUser'}, 'sessionId': 'default-snowboy-52460f67-54eb-45cd-b619-748cf8453b0d', 'siteId': 'default', 'customData': 'snowboy'}



# all test sequences performed with a hotword wake up
test_sequence_wake_up_sequence_intent_recognized = [
	TestData('wake_up', 'WaitingRhasspySession', WAKE_UP_MSG_1, FROM_WAIT_RH_NON_POSSIBLE),         # Wake up
	TestData('session_started', 'SessionActive', SESS_STARTED_MSG_1, FROM_SESS_ACT_NON_POSSIBLE),   # simulate a session started reicived
	TestData('asr_started', 'WaitingSpeech', START_LISTENING_MSG_1, FROM_WAIT_SPCH_NON_POSSIBLE),   # Going in Waiting Speech
	TestData('text_captured', 'WaitingIntent', TEXT_CAPTURED_MSG_1, FROM_WAIT_INTENT_NON_POSSIBLE), # Process Intent
	TestData('intent_recognized', 'SessionActive', INTENT_RECOGN_MSG_1, FROM_SESS_ACT_NON_POSSIBLE),# getting intent_recognized
	TestData('rh_session_reset', 'WaitingRhasspySession', RH_RESET_SESS_MSG_1, FROM_WAIT_RH_NON_POSSIBLE), # simulate a closing session
	TestData('rh_session_reset', 'WaitingRhasspySession', RH_RESET_SESS_DURING_WAIT_MSG_1, FROM_WAIT_RH_NON_POSSIBLE), # simulate a closing session when in waiting, this should be possible
] #TODO: add say!!!

test_sequence_wake_up_sequence_intent_not_recognized = [
	TestData('wake_up', 'WaitingRhasspySession', WAKE_UP_MSG_1, FROM_WAIT_RH_NON_POSSIBLE),         # Wake up
	TestData('session_started', 'SessionActive', SESS_STARTED_MSG_1, FROM_SESS_ACT_NON_POSSIBLE),   # simulate a session started reicived
	TestData('asr_started', 'WaitingSpeech', START_LISTENING_MSG_1, FROM_WAIT_SPCH_NON_POSSIBLE),   # Going in Waiting Speech
	TestData('text_captured', 'WaitingIntent', TEXT_CAPTURED_MSG_1, FROM_WAIT_INTENT_NON_POSSIBLE), # Process Intent
	TestData('intent_not_recognized', 'SessionActive', INTENT_NOT_RECOGN_MSG_1, FROM_SESS_ACT_NON_POSSIBLE),# simulate intent not recognized
	TestData('rh_session_reset', 'WaitingRhasspySession', RH_RESET_SESS_MSG_1, FROM_WAIT_RH_NON_POSSIBLE), # simulate a closing session
	TestData('rh_session_reset', 'WaitingRhasspySession', RH_RESET_SESS_DURING_WAIT_MSG_1, FROM_WAIT_RH_NON_POSSIBLE), # simulate a closing session when in waiting, this should be possible
]

test_sequence_post_sequence_intent_recognized = [
	TestData('session_started', 'SessionActive', SESS_STARTED_MSG_1, FROM_SESS_ACT_NON_POSSIBLE),   # simulate a session started reicived
	TestData('asr_started', 'WaitingSpeech', START_LISTENING_MSG_1, FROM_WAIT_SPCH_NON_POSSIBLE),   # Going in Waiting Speech
	TestData('text_captured', 'WaitingIntent', TEXT_CAPTURED_MSG_1, FROM_WAIT_INTENT_NON_POSSIBLE), # Process Intent
	TestData('intent_recognized', 'SessionActive', INTENT_RECOGN_MSG_1, FROM_SESS_ACT_NON_POSSIBLE),# getting intent_recognized
	TestData('rh_session_reset', 'WaitingRhasspySession', RH_RESET_SESS_MSG_1, FROM_WAIT_RH_NON_POSSIBLE), # simulate a closing session
]

test_sequence_post_sequence_intent_not_recognized = [
	TestData('session_started', 'SessionActive', SESS_STARTED_MSG_1, FROM_SESS_ACT_NON_POSSIBLE),   # simulate a session started reicived
	TestData('asr_started', 'WaitingSpeech', START_LISTENING_MSG_1, FROM_WAIT_SPCH_NON_POSSIBLE),   # Going in Waiting Speech
	TestData('text_captured', 'WaitingIntent', TEXT_CAPTURED_MSG_1, FROM_WAIT_INTENT_NON_POSSIBLE), # Process Intent
	TestData('intent_not_recognized', 'SessionActive', INTENT_NOT_RECOGN_MSG_1, FROM_SESS_ACT_NON_POSSIBLE),# simulate intent not recognized
	TestData('rh_session_reset', 'WaitingRhasspySession', RH_RESET_SESS_MSG_1, FROM_WAIT_RH_NON_POSSIBLE), # simulate a closing session
]

test_sequence_wake_up_interrupt = test_sequence_post_sequence_intent_not_recognized

test_sequence_close_session = [
	TestData('session_started', 'SessionActive', SESS_STARTED_MSG_1, FROM_SESS_ACT_NON_POSSIBLE),   # simulate a session started reicived
	TestData('asr_started', 'WaitingSpeech', START_LISTENING_MSG_1, FROM_WAIT_SPCH_NON_POSSIBLE),   # Going in Waiting Speech
	TestData('text_captured', 'WaitingIntent', TEXT_CAPTURED_MSG_1, FROM_WAIT_INTENT_NON_POSSIBLE), # Process Intent
	TestData('intent_not_recognized', 'SessionActive', INTENT_NOT_RECOGN_MSG_1, FROM_SESS_ACT_NON_POSSIBLE),# simulate intent not recognized				 
]

test_sequence_mix_session_data = [
	TestData('asr_started', 'WaitingSpeech', START_LISTENING_MSG_1, FROM_WAIT_SPCH_NON_POSSIBLE),   # Going in Waiting Speech
	TestData('text_captured', 'WaitingIntent', TEXT_CAPTURED_MSG_1, FROM_WAIT_INTENT_NON_POSSIBLE), # Process Intent
	TestData('intent_not_recognized', 'SessionActive', INTENT_NOT_RECOGN_MSG_1, FROM_SESS_ACT_NON_POSSIBLE),# simulate intent not recognized				 
	TestData('rh_session_reset', 'WaitingRhasspySession', RH_RESET_SESS_MSG_1, FROM_WAIT_RH_NON_POSSIBLE), # simulate a closing session
]


########################################################
## DEFINITION FOR TestLisaStateMachineExternalRequest ##
########################################################
# Example of messages from a real session
EXTERNAL_MSG_NO_SESSION_2A = {'externalSessionID': None, 'siteId': 'default'}
EXTERNAL_MSG_WITH_SESSION_2B = {'externalSessionID': 'SomeExternalSession_BLABLA', 'siteId': 'default'}
SAY_MSG_2 = {'text': 'Ask me about the time',   'siteId': 'default', 'lang': None, 'id': '8cac8399-5ac6-4037-90f2-7149251cd575', 'sessionId': '0a9d4d16-0808-4db0-80d8-2d11e75f5b40'}
TTS_SAY_FINISHED_2 = {'siteId': 'default', 'id': '8cac8399-5ac6-4037-90f2-7149251cd575', 'sessionId': '0a9d4d16-0808-4db0-80d8-2d11e75f5b40'}
SESS_STARTED_MSG_2 = {'sessionId': '0a9d4d16-0808-4db0-80d8-2d11e75f5b40', 'siteId': 'default', 'customData': 'Anonymous_1', 'lang': None}	
START_LISTENING_MSG_2 ={'siteId': 'default', 'sessionId': '0a9d4d16-0808-4db0-80d8-2d11e75f5b40', 'lang': None, 'stopOnSilence': True, 'sendAudioCaptured': True, 'wakewordId': '', 'intentFilter': None}
TEXT_CAPTURED_MSG_2 = {'text': 'what time is it', 'likelihood': 1, 'seconds': 0.7641713820012228, 'siteId': 'default', 'sessionId': '0a9d4d16-0808-4db0-80d8-2d11e75f5b40', 'wakewordId': None, 'asrTokens': None, 'lang': None}
INTENT_RECOGN_MSG_2 = {'input': 'what time is it', 'intent': {'intentName': 'GetTime', 'confidenceScore': 1.0}, 'siteId': 'default', 'id': None, 'slots': [], 'sessionId': '0a9d4d16-0808-4db0-80d8-2d11e75f5b40', 'customData': None, 'asrTokens': [[{'value': 'what', 'confidence': 1.0, 'rangeStart': 0, 'rangeEnd': 4, 'time': None}, {'value': 'time', 'confidence': 1.0, 'rangeStart': 5, 'rangeEnd': 9, 'time': None}, {'value': 'is', 'confidence': 1.0, 'rangeStart': 10, 'rangeEnd': 12, 'time': None}, {'value': 'it', 'confidence': 1.0, 'rangeStart': 13, 'rangeEnd': 15, 'time': None}]], 'asrConfidence': None, 'rawInput': 'what time is it', 'wakewordId': '', 'lang': None}
INTENT_NOT_RECOGN_MSG_2 = {'input': 'is it', 'siteId': 'default', 'id': None, 'customData': None, 'sessionId': '0a9d4d16-0808-4db0-80d8-2d11e75f5b40'}
RH_RESET_SESS_NOT_RECOGN_MSG_2 = {'termination': {'reason': 'intentNotRecognized'}, 'sessionId': '0a9d4d16-0808-4db0-80d8-2d11e75f5b40', 'siteId': 'default', 'customData': 'Anonymous_1'}
RH_RESET_SESS_TIMEOUT_MSG_2 = {'termination': {'reason': 'timeout'}, 'sessionId': '0a9d4d16-0808-4db0-80d8-2d11e75f5b40', 'siteId': 'default', 'customData': 'Anonymous_1'}
RH_RESET_SESS_RECOGN_MSG_2 = {'termination': {'reason': 'nominal'}, 'sessionId': '0a9d4d16-0808-4db0-80d8-2d11e75f5b40', 'siteId': 'default', 'customData': 'Anonymous_1'}
RH_RESET_SESS_DURING_WAIT_MSG_2 = {'termination': {'reason': 'abortedByUser'}, 'sessionId': '0a9d4d16-0808-4db0-80d8-2d11e75f5b40', 'siteId': 'default', 'customData': 'Anonymous_1'}

# Different session and customData
EXTERNAL_MSG_NO_SESSION_3A = {'externalSessionID': None, 'siteId': 'default'}
EXTERNAL_MSG_WITH_SESSION_3B = {'externalSessionID': 'SomeExternalSession_BLABLA', 'siteId': 'default'}
SAY_MSG_3 = {'text': 'Ask me about the time',   'siteId': 'default', 'lang': None, 'id': '8cac8399-5ac6-4037-90f2-7149251cd575', 'sessionId': '52460f67-4db0-45cd-b619-748cf8453b0d'}
TTS_SAY_FINISHED_3 = {'siteId': 'default', 'id': '8cac8399-5ac6-4037-90f2-7149251cd575', 'sessionId': '52460f67-4db0-45cd-b619-748cf8453b0d'}
SESS_STARTED_MSG_3 = {'sessionId': '52460f67-4db0-45cd-b619-748cf8453b0d', 'siteId': 'default', 'customData': 'Some_session', 'lang': None}	
START_LISTENING_MSG_3 ={'siteId': 'default', 'sessionId': '52460f67-4db0-45cd-b619-748cf8453b0d', 'lang': None, 'stopOnSilence': True, 'sendAudioCaptured': True, 'wakewordId': '', 'intentFilter': None}
TEXT_CAPTURED_MSG_3 = {'text': 'what time is it', 'likelihood': 1, 'seconds': 0.7641713820012228, 'siteId': 'default', 'sessionId': '52460f67-4db0-45cd-b619-748cf8453b0d', 'wakewordId': None, 'asrTokens': None, 'lang': None}
INTENT_RECOGN_MSG_3 = {'input': 'what time is it', 'intent': {'intentName': 'GetTime', 'confidenceScore': 1.0}, 'siteId': 'default', 'id': None, 'slots': [], 'sessionId': '52460f67-4db0-45cd-b619-748cf8453b0d', 'customData': None, 'asrTokens': [[{'value': 'what', 'confidence': 1.0, 'rangeStart': 0, 'rangeEnd': 4, 'time': None}, {'value': 'time', 'confidence': 1.0, 'rangeStart': 5, 'rangeEnd': 9, 'time': None}, {'value': 'is', 'confidence': 1.0, 'rangeStart': 10, 'rangeEnd': 12, 'time': None}, {'value': 'it', 'confidence': 1.0, 'rangeStart': 13, 'rangeEnd': 15, 'time': None}]], 'asrConfidence': None, 'rawInput': 'what time is it', 'wakewordId': '', 'lang': None}
INTENT_NOT_RECOGN_MSG_3 = {'input': 'is it', 'siteId': 'default', 'id': None, 'customData': None, 'sessionId': '52460f67-4db0-45cd-b619-748cf8453b0d'}
RH_RESET_SESS_NOT_RECOGN_MSG_3 = {'termination': {'reason': 'intentNotRecognized'}, 'sessionId': '52460f67-4db0-45cd-b619-748cf8453b0d', 'siteId': 'default', 'customData': 'Some_session'}
RH_RESET_SESS_TIMEOUT_MSG_3 = {'termination': {'reason': 'timeout'}, 'sessionId': '52460f67-4db0-45cd-b619-748cf8453b0d', 'siteId': 'default', 'customData': 'Some_session'}
RH_RESET_SESS_RECOGN_MSG_3 = {'termination': {'reason': 'nominal'}, 'sessionId': '52460f67-4db0-45cd-b619-748cf8453b0d', 'siteId': 'default', 'customData': 'Some_session'}
RH_RESET_SESS_DURING_WAIT_MSG_3 = {'termination': {'reason': 'abortedByUser'}, 'sessionId': '52460f67-4db0-45cd-b619-748cf8453b0d', 'siteId': 'default', 'customData': 'Some_session'}


# all test sequences performed with programmatic call (external request) 

test_sequence_notification_only = [
	# It seems with notification a sessionStarted is not broadcasted
	TestData('tts_say', 'WaitingSay', SAY_MSG_2, FROM_WAIT_SAY_NON_POSSIBLE), 
	TestData('tts_finished', 'SessionActive', TTS_SAY_FINISHED_2, FROM_SESS_ACT_NON_POSSIBLE), 
	# A reset or a close session is never really called. This could be a problemt?	
	TestData('rh_session_reset', 'WaitingRhasspySession', RH_RESET_SESS_TIMEOUT_MSG_2, FROM_WAIT_RH_NON_POSSIBLE), # simulate a closing session
]

test_sequence_external_request_notifcation_only_no_ext_id = [
	TestData('external_request', 'WaitingRhasspySession', EXTERNAL_MSG_NO_SESSION_2A, FROM_WAIT_RH_NON_POSSIBLE), # External Request
] + test_sequence_notification_only

test_sequence_external_request_notifcation_only_with_ext_id = [
	TestData('external_request', 'WaitingRhasspySession', EXTERNAL_MSG_WITH_SESSION_2B, FROM_WAIT_RH_NON_POSSIBLE), # External Request
] + test_sequence_notification_only

# WITH SAY!!
test_sequence_external_request_to_intent_not_recognized = [
	TestData('external_request', 'WaitingRhasspySession', EXTERNAL_MSG_NO_SESSION_2A, FROM_WAIT_RH_NON_POSSIBLE),        # External Request
	TestData('session_started', 'SessionActive', SESS_STARTED_MSG_2, FROM_SESS_ACT_NON_POSSIBLE),   # simulate a session started reicived
	TestData('tts_say', 'WaitingSay', SAY_MSG_2, FROM_WAIT_SAY_NON_POSSIBLE), 
	TestData('tts_finished', 'SessionActive', TTS_SAY_FINISHED_2, FROM_SESS_ACT_NON_POSSIBLE), 
	TestData('asr_started', 'WaitingSpeech', START_LISTENING_MSG_2, FROM_WAIT_SPCH_NON_POSSIBLE),   # Going in Waiting Speech
	TestData('text_captured', 'WaitingIntent', TEXT_CAPTURED_MSG_2, FROM_WAIT_INTENT_NON_POSSIBLE), # Process Intent
	TestData('intent_not_recognized', 'SessionActive', INTENT_NOT_RECOGN_MSG_2, FROM_SESS_ACT_NON_POSSIBLE),# simulate intent not recognized
	TestData('rh_session_reset', 'WaitingRhasspySession', RH_RESET_SESS_NOT_RECOGN_MSG_2, FROM_WAIT_RH_NON_POSSIBLE), # simulate a closing session
	TestData('rh_session_reset', 'WaitingRhasspySession', RH_RESET_SESS_DURING_WAIT_MSG_2, FROM_WAIT_RH_NON_POSSIBLE), # simulate a closing session when in waiting, this should be possible
]

test_sequence_external_request_to_intent_recognized = [
	TestData('external_request', 'WaitingRhasspySession', EXTERNAL_MSG_NO_SESSION_3A, FROM_WAIT_RH_NON_POSSIBLE),        # External Request
	TestData('session_started', 'SessionActive', SESS_STARTED_MSG_3, FROM_SESS_ACT_NON_POSSIBLE),   # simulate a session started reicived
	TestData('tts_say', 'WaitingSay', SAY_MSG_3, FROM_WAIT_SAY_NON_POSSIBLE), 
	TestData('tts_finished', 'SessionActive', TTS_SAY_FINISHED_3, FROM_SESS_ACT_NON_POSSIBLE), 
	TestData('asr_started', 'WaitingSpeech', START_LISTENING_MSG_3, FROM_WAIT_SPCH_NON_POSSIBLE),   # Going in Waiting Speech
	TestData('text_captured', 'WaitingIntent', TEXT_CAPTURED_MSG_3, FROM_WAIT_INTENT_NON_POSSIBLE), # Process Intent
	TestData('intent_recognized', 'SessionActive', INTENT_RECOGN_MSG_3, FROM_SESS_ACT_NON_POSSIBLE),# getting intent_recognized
	TestData('rh_session_reset', 'WaitingRhasspySession', RH_RESET_SESS_RECOGN_MSG_3, FROM_WAIT_RH_NON_POSSIBLE), # simulate a closing session	
	TestData('rh_session_reset', 'WaitingRhasspySession', RH_RESET_SESS_DURING_WAIT_MSG_2, FROM_WAIT_RH_NON_POSSIBLE), # simulate a closing session when in waiting, this should be possible
]

test_sequence_external_request_interrupt = test_sequence_external_request_to_intent_recognized

# ----------------------
# --- UTILITIES FUNC ---
# ---------------------- 
def get_session():
	session = DialogueSession()
	dialogue_sm = Machine(model=session, states=states, transitions=transitions, initial=INIT_STATE)
	return session, dialogue_sm 
	

def header_str(title, line_prefix = ''):
	title = " " + title + " "
	len_title = len(title)
	return line_prefix + "-"*(len_title+10) + "\n"+ line_prefix + "-"*5 + title +"-"*5 + "\n"+line_prefix+"-"*(len_title+10)

	
def test_case_str(title):
	return "\n\n\n"+"*"*80+" \n" + header_str(title) 


# ------------
# --- TEST ---
# ------------ 

class TestLisaStateMachineBaseClass(unittest.TestCase):

	@classmethod
	def setUp(cls):
		pass

	@classmethod
	def tearDownClass(cls):
		pass
	
	def _new_session(self):
		self.session = DialogueSession()
		self.dialogue_sm = Machine(model=self.session, states=states, transitions=transitions, initial=INIT_STATE)
	
	def _call_non_possible_transition(self, non_possible_trans, msg = None):
		for trans in non_possible_trans:
			print('{:_<80.77}'.format('\t\tFrom '+ str(self.session.state)+' call ' + trans), end='\t ')
			with self.assertRaises(MachineError) as cm:
				try:
					getattr(self.session, trans)(payload = msg)
				except Exception as e:
					if str(e).startswith("\"Can't trigger event"):
						print("Pass")
					else:
						print("Fail: " + str(e))
					raise e
	
	def _call_transition_full_check(self, transition, next_state, msg, non_possible_transition):
		self._call_transition(transition, next_state, msg)
		print("\tChecking all non possible transitions",)
		self._call_non_possible_transition(non_possible_transition)
		print("\tAll transitions checked")
		
	def _call_transition(self, transition, next_state, msg):
		print(header_str("From state {} - call transition {} - To state {}".format(self.session.state, transition, next_state), line_prefix='\t'))
		getattr(self.session, transition)(payload = msg)
		print("\tCalling next transition ... ", end='\t')
		assert self.session.state == next_state, "Expected next_state={}, received instead={}".format(next_state, self.session.state )
		print("OK")

	def _call_and_check_transition_sequence(self, transition):
		for t in transition:
			self._call_transition_full_check(t.transition, t.next_state, t.msg, t.non_possible_states)
			
	def _call_and_check_transition_sequence_with_interreupt(self, test_sequence, interrupt_transition):
		# Try to call the interrupt transition after every state in test_sequence
		done = []
		for t in test_sequence:
			self._call_transition_full_check(t.transition, t.next_state, t.msg, t.non_possible_states)
			self._call_transition(interrupt_transition.transition, interrupt_transition.next_state, interrupt_transition.msg)
			# Restore the point
			done.append(t)
			for d in done:
				self._call_transition(d.transition, d.next_state, d.msg)
			
			
class TestLisaStateMachineWakeUpWord(TestLisaStateMachineBaseClass):

	def test_init_state(self):
		# Remember to call at the begin of every test
		print(test_case_str('*** Start test_init_state ***'))
		self._new_session()
		# Check init state behaviour
		assert self.session.state == 'Init', 'Wrong initial state: ' + str(self.session.state)
		print(header_str('Check Init transitions' + str(self.session.state), line_prefix='\t'))
		self._call_non_possible_transition(FROM_INIT_NON_POSSIBLE)
	
	def test_wake_up_sequence_intent_recognized(self):
		"""
		Test a sequence simulating from a wake up to a non recognized event
		Try also to call all the other non possible state as test
		"""
		print(test_case_str('*** Start test_wake_up_sequence_intent_recognized ***'))
		
		# Remember to call at the begin of every test
		self._new_session()
		
		# iterate
		test_sequence = test_sequence_wake_up_sequence_intent_recognized + \
						test_sequence_post_sequence_intent_not_recognized + \
						test_sequence_post_sequence_intent_recognized + \
						test_sequence_wake_up_sequence_intent_not_recognized
		self._call_and_check_transition_sequence(test_sequence)
		#for t in test_sequence_wake_up_sequence_intent_recognized:
		#	self._call_transition_full_check(t.transition, t.next_state, t.msg, t.non_possible_states)
	
	def test_wake_up_sequence_intent_not_recognized(self):
		"""
		Test a sequence simulating from a wake up to a non recognized event
		Try also to call all the other non possible state as test
		"""
		print(test_case_str('*** Start test_wake_up_sequence_intent_not_recognized ***'))
		
		# Remember to call at the begin of every test
		self._new_session()
		test_sequence = test_sequence_wake_up_sequence_intent_not_recognized + \
						test_sequence_post_sequence_intent_recognized + \
						test_sequence_post_sequence_intent_not_recognized + \
						test_sequence_wake_up_sequence_intent_recognized
		self._call_and_check_transition_sequence(test_sequence)
		# for t in test_sequence:
			# self._call_transition_full_check(t.transition, t.next_state, t.msg, t.non_possible_states)

	def test_wake_up_interrupt(self):
		"""
		Try to call a wake from every designed state
		"""
		print(test_case_str('*** Start test_wake_up_interrupt ***'))
	
		# Remember to call at the begin of every test
		self._new_session()
		# Wake up, we want to move to this transition
		wake_up_intp = TestData('wake_up', 'WaitingRhasspySession', WAKE_UP_MSG_1, FROM_WAIT_RH_NON_POSSIBLE)
		self._call_transition(wake_up_intp.transition, wake_up_intp.next_state, wake_up_intp.msg)
		self._call_and_check_transition_sequence_with_interreupt(test_sequence_wake_up_interrupt, wake_up_intp)
	
	def test_close_session(self):
		"""
		Try to close the session from every designed state
		"""
		print(test_case_str('*** Start test_close_session ***'))
		self._new_session() # Remember to call at the begin of every test
		
		# Wake up, we want to move to this transition
		self._call_transition('wake_up', 'WaitingRhasspySession', WAKE_UP_MSG_1)
		# Prepare a close TestData
		close_sess_intp = TestData('rh_session_reset', 'WaitingRhasspySession', RH_RESET_SESS_MSG_1, FROM_WAIT_RH_NON_POSSIBLE) # simulate a closing session		
		self._call_and_check_transition_sequence_with_interreupt(test_sequence_close_session, close_sess_intp)

	def test_mix_session_data(self):
		"""
		Execute calls with different ID, rhasspy session and site. Check the state change is refused
		"""
		print(test_case_str('*** Start test_mix_session_data ***'))
		self._new_session() # Remember to call at the begin of every test
		
		# Remember to call at the begin of every test
		self._new_session()
		
		# moving to wake up
		self._call_transition('wake_up', 'WaitingRhasspySession', WAKE_UP_MSG_1)
		# self._call_transition('session_started', 'SessionActive', SESS_STARTED_MSG_1)
		
		# This transction raise an exception only for wrong siteId
		sess_active = TestData('session_started', 'SessionActive', SESS_STARTED_MSG_1, FROM_SESS_ACT_NON_POSSIBLE)   # simulate a session started reicived
		msg_rh_site = dict(sess_active.msg)
		msg_rh_site['siteId'] = 'bbbb' 
		with self.assertRaises(DialogueSessionException) as ds:
			self._call_transition(sess_active.transition, sess_active.next_state, msg_rh_site)
			
		# finally call it
		self._call_transition_full_check(sess_active.transition, sess_active.next_state, sess_active.msg, sess_active.non_possible_states)
		
		# iterate
		for t in test_sequence_mix_session_data:
			# TODO: session ID is not yet verified
			# session_ID = 
			msg_rh_sess = dict(t.msg)
			msg_rh_site = dict(t.msg)
			msg_rh_sess['sessionId'] = 'aaaa'
			msg_rh_site['siteId'] = 'bbbb'
			
			# this will raise an error
			with self.assertRaises(DialogueSessionException) as ds:
				self._call_transition(t.transition, t.next_state, msg_rh_site)
				
			with self.assertRaises(DialogueSessionException) as ds:
				self._call_transition(t.transition, t.next_state, msg_rh_sess, )
			
			# perform state change
			self._call_transition_full_check(t.transition, t.next_state, t.msg, t.non_possible_states)
		
		# This transaction raise an exception only for wrong siteId
		reset_session = TestData('rh_session_reset', 'WaitingRhasspySession', RH_RESET_SESS_MSG_1, FROM_WAIT_RH_NON_POSSIBLE)
		msg_rh_site = dict(reset_session.msg)
		msg_rh_site['siteId'] = 'ddd' 
		with self.assertRaises(DialogueSessionException) as ds:
			self._call_transition(reset_session.transition, reset_session.next_state, msg_rh_site)
			
		# finally call it
		self._call_transition_full_check(reset_session.transition, reset_session.next_state, reset_session.msg, reset_session.non_possible_states)
		
	def test_different_wakeup_word(self):
		print(test_case_str('*** Start test_mix_session_data ***'))
		self._new_session() # Remember to call at the begin of every test
				
		# self._call_transition('session_started', 'SessionActive', SESS_STARTED_MSG_1)
		
		# Change the wake up word
		wakep_word_detected = TestData('wake_up', 'WaitingRhasspySession', WAKE_UP_MSG_1, FROM_WAIT_RH_NON_POSSIBLE) 
		msg_wakeup = dict(wakep_word_detected.msg)
		msg_wakeup['modelId'] = msg_wakeup['modelId']+'ccc'
		with self.assertRaises(DialogueException) as ds:
			self._call_transition(wakep_word_detected.transition, wakep_word_detected.next_state, msg_wakeup)	
		self._call_transition_full_check(wakep_word_detected.transition, wakep_word_detected.next_state, wakep_word_detected.msg, wakep_word_detected.non_possible_states)


class TestLisaStateMachineExternalRequest(TestLisaStateMachineBaseClass):

	def test_init_state(self):
		# Remember to call at the begin of every test
		print(test_case_str('*** Start test_init_state ***'))
		self._new_session()
		# Check init state behaviour
		assert self.session.state == 'Init', 'Wrong initial state: ' + str(self.session.state)
		print(header_str('Check Init transitions' + str(self.session.state), line_prefix='\t'))
		self._call_non_possible_transition(FROM_INIT_NON_POSSIBLE)
	
	
	def test_external_request_notification_only(self):
		"""
		Test a an external notification (speech) only. With and without given external sessionID
		Try also to call all the other non possible state as test
		"""
		
		# Remember to call at the begin of every test
		self._new_session()
		print(test_case_str('*** Start test_external_request_notification_only ***'))
		
		# test in the first order
		print('Test first sequence')
		test_sequence = test_sequence_external_request_notifcation_only_no_ext_id + test_sequence_external_request_notifcation_only_with_ext_id +\
						test_sequence_external_request_notifcation_only_with_ext_id + test_sequence_external_request_notifcation_only_no_ext_id
		self._call_and_check_transition_sequence(test_sequence)
		
		# test in another order
		self._new_session()
		print('Test second sequence')
		test_sequence = test_sequence_external_request_notifcation_only_with_ext_id + test_sequence_external_request_notifcation_only_no_ext_id +\
						test_sequence_external_request_notifcation_only_no_ext_id + test_sequence_external_request_notifcation_only_with_ext_id 
		self._call_and_check_transition_sequence(test_sequence)

	def test_external_request_sequence_intent_recognized(self):
		"""
		Test a sequence simulating from an external request to a non recognized event
		Try also to call all the other non possible state as test
		"""
		print(test_case_str('*** Start test_external_request_sequence_intent_recognized ***'))
		
		# Remember to call at the begin of every test
		self._new_session()

		test_sequence = test_sequence_external_request_to_intent_recognized +\
						test_sequence_external_request_notifcation_only_no_ext_id+\
						test_sequence_external_request_to_intent_recognized +\
						test_sequence_external_request_notifcation_only_no_ext_id
		self._call_and_check_transition_sequence(test_sequence)
	
	def test_external_request_sequence_intent_not_recognized(self):
		"""
		Test a sequence simulating from an external request to a non recognized event
		Try also to call all the other non possible state as test
		"""
		print(test_case_str('*** Start test_external_request_sequence_intent_not_recognized ***'))
		
		# Remember to call at the begin of every test
		self._new_session()
		test_sequence = test_sequence_external_request_to_intent_not_recognized +\
						test_sequence_post_sequence_intent_recognized
		self._call_and_check_transition_sequence(test_sequence)

	def test_external_request_interrupt(self):
		"""
		Try to call an external request interrupt from every designed state
		"""
		print(test_case_str('*** Start test_external_request_interrupt ***'))
	
		# Remember to call at the begin of every test
		self._new_session()
		# Wake up, we want to move to this transition
		ext_intrp = TestData('external_request', 'WaitingRhasspySession', EXTERNAL_MSG_NO_SESSION_2A, FROM_WAIT_RH_NON_POSSIBLE)
		self._call_transition(ext_intrp.transition, ext_intrp.next_state, ext_intrp.msg)
		self._call_and_check_transition_sequence_with_interreupt(test_sequence_external_request_interrupt, ext_intrp)
		

class TestLisaStateMachineMixRequests(TestLisaStateMachineBaseClass):
	
	def test_random_mix_sequences(self):
		"""
		Test a an external notification (speech) only. With and without given external sessionID
		Try also to call all the other non possible state as test
		"""
		from random import randrange
		# Remember to call at the begin of every test
		self._new_session()
		print(test_case_str('*** Start test_external_request_notification_only ***'))
		
		n_iteration = 30
		sequences = [test_sequence_wake_up_sequence_intent_recognized, 
					 test_sequence_wake_up_sequence_intent_not_recognized,
					 test_sequence_external_request_to_intent_not_recognized,
					 test_sequence_external_request_to_intent_recognized]
		n_seq = len(sequences)
		test_sequence = []
		for n in range(n_iteration):
			seq = sequences[randrange(0,n_seq,1)]
			interruption_index =  randrange(1,len(seq),1)
			test_sequence.append(seq[:interruption_index])
		test_sequence = [item for subseq in test_sequence for item in subseq]
		
		print('Test list is made of {} transitions'.format(len(test_sequence)))
		print("|Init| --- " + "".join(["--- ({}) --- |{}| ".format(t.transition, t.next_state) for t in test_sequence]))
		self._call_and_check_transition_sequence(test_sequence)
	

if __name__ == '__main__':
	import rosunit
	rosunit.unitrun("lisa_mqtt_ros_bridge", "test_state_machine_wake_up_word", TestLisaStateMachineWakeUpWord)
	rosunit.unitrun("lisa_mqtt_ros_bridge", "test_state_machine_external_request", TestLisaStateMachineExternalRequest)
	rosunit.unitrun("lisa_mqtt_ros_bridge", "test_state_machine_mix_request", TestLisaStateMachineMixRequests)
