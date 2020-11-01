#!/usr/bin/env python3

import unittest
from collections import namedtuple

from lisa_mqtt.dialogue import DialogueSession, Machine, MachineError, states, transitions, INIT_STATE, DialogueSessionException

# Which transitions are NON possible from a specific state. Calling a transition in that state would cause an error
#ALL STATES				= 	    ['wake_up', 'external_request', 'session_started', 'asr_started', 'text_captured','intent_recognized','intent_not_recognized','rh_session_reset' ]
FROM_INIT_NON_POSSIBLE =	 	[                               'session_started', 'asr_started', 'text_captured', 'intent_recognized', 'intent_not_recognized','rh_session_reset' ]
FROM_WAIT_RH_NON_POSSIBLE  =	[          'external_request',     				   'asr_started', 'text_captured', 'intent_recognized', 'intent_not_recognized' ]
FROM_SESS_ACT_NON_POSSIBLE =	[          'external_request', 	'session_started', 				  'text_captured', 'intent_recognized', 'intent_not_recognized' ]
FROM_WAIT_SPCH_NON_POSSIBLE	=	[          'external_request',  'session_started', 'asr_started',                  'intent_recognized', 'intent_not_recognized' ]
FROM_WAIT_INTENT_NON_POSSIBLE = [          'external_request',  'session_started', 'asr_started', 'text_captured',]

# Defined data structures
# TODO: sessionId is not checked, this messages has to be modified then!!
WAKE_UP_MSG_1 = {'modelId': 'snowboy', 'modelVersion': '', 'modelType': 'personal', 'currentSensitivity': 0.8, 'siteId': 'default', 'sessionId': None, 'sendAudioCaptured': None, 'lang': None,} # 'termination': {'reason': 'intentNotRecognized'}}
SESS_STARTED_MSG_1 = {'sessionId': 'default-snowboy-261df2bc-8e52-4313-b98a-89fe11fdeead', 'siteId': 'default', 'customData': 'snowboy', 'lang': None}
START_LISTENING_MSG_1 = {'siteId': 'default', 'sessionId': 'default-snowboy-261df2bc-8e52-4313-b98a-89fe11fdeead', 'lang': None, 'stopOnSilence': True, 'sendAudioCaptured': True, 'wakewordId': 'snowboy', 'intentFilter': None}
TEXT_CAPTURED_MSG_1 = {'text': 'what time is it', 'likelihood': 1, 'seconds': 1.3038434779991803, 'siteId': 'default', 'sessionId': 'default-snowboy-52460f67-54eb-45cd-b619-748cf8453b0d', 'wakewordId': None, 'asrTokens': None, 'lang': None}
INTENT_RECOGN_MSG_1 = {'input': 'what time is it', 'intent': {'intentName': 'GetTime', 'confidenceScore': 1.0}, 'siteId': 'default', 'id': None, 'slots': [], 'sessionId': 'default-snowboy-52460f67-54eb-45cd-b619-748cf8453b0d'}
INTENT_NOT_RECOGN_MSG_1 = {'input': 'set blue', 'siteId': 'default', 'id': None, 'customData': None, 'sessionId': 'default-snowboy-bdd769e2-7816-4ee1-b245-70f46008de63'}
RH_RESET_SESS_MSG_1 = {'termination': {'reason': 'timeout'}, 'sessionId': 'default-snowboy-52460f67-54eb-45cd-b619-748cf8453b0d', 'siteId': 'default', 'customData': 'snowboy'}


# Used as test data in sequence
TestData = namedtuple('TestData', ['transition', 'next_state', 'msg', 'non_possible_states'])

def get_session():
	session = DialogueSession()
	dialogue_sm = Machine(model=session, states=states, transitions=transitions, initial=INIT_STATE)
	return session, dialogue_sm 
	

def header_str(title):
	title = " " + title + " "
	len_title = len(title)
	return "-"*(len_title+10) + "\n" + "-"*5 + title +"-"*5 + "\n"+ "-"*(len_title+10)

	
def test_case_str(title):
	return "\n\n\n"+"*"*80+" \n" + header_str(title) 


class TestStateMachineWakeUp(unittest.TestCase):

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
		#print(header_str("From state {} - call transition {} - To state {}".format(self.session.state, transition, next_state)))
		#getattr(self.session, transition)(payload = msg)
		#print("\tCalling next transition ... ", end='\t')
		#assert self.session.state == next_state
		#print("OK")
		
		self._call_transition(transition, next_state, msg)
		print("\tChecking all non possible transitions",)
		self._call_non_possible_transition(non_possible_transition)
		print("\tAll transitions checked")
		
	def _call_transition(self, transition, next_state, msg):
		print(header_str("From state {} - call transition {} - To state {}".format(self.session.state, transition, next_state)))
		getattr(self.session, transition)(payload = msg)
		print("\tCalling next transition ... ", end='\t')
		assert self.session.state == next_state
		print("OK")
		
	
	def test_init_state(self):
		# Remember to call at the begin of every test
		print(test_case_str('*** Start test_init_state ***'))
		self._new_session()
		# Check init state behaviour
		assert self.session.state == 'Init', 'Wrong initial state: ' + str(self.session.state)
		print(header_str('Check Init transitions' + str(self.session.state)))
		self._call_non_possible_transition(FROM_INIT_NON_POSSIBLE)
	
	def test_wake_up_sequence_intent_recognized(self):
		"""
		Test a sequence simulating from a wake up to a non recognized event
		Try also to call all the other non possible state as test
		"""
		print(test_case_str('*** Start test_wake_up_sequence_intent_recognized ***'))
		
		# Remember to call at the begin of every test
		self._new_session()
		test_sequence = [TestData('wake_up', 'WaitingRhasspySession', WAKE_UP_MSG_1, FROM_WAIT_RH_NON_POSSIBLE),         # Wake up
						 TestData('session_started', 'SessionActive', SESS_STARTED_MSG_1, FROM_SESS_ACT_NON_POSSIBLE),   # simulate a session started reicived
						 TestData('asr_started', 'WaitingSpeech', START_LISTENING_MSG_1, FROM_WAIT_SPCH_NON_POSSIBLE),   # Going in Waiting Speech
						 TestData('text_captured', 'WaitingIntent', TEXT_CAPTURED_MSG_1, FROM_WAIT_INTENT_NON_POSSIBLE), # Process Intent
						 TestData('intent_recognized', 'SessionActive', INTENT_RECOGN_MSG_1, FROM_SESS_ACT_NON_POSSIBLE),# getting intent_recognized
						 TestData('rh_session_reset', 'WaitingRhasspySession', RH_RESET_SESS_MSG_1, FROM_WAIT_RH_NON_POSSIBLE), # simulate a closing session
						]
		for t in test_sequence:
			self._call_transition_full_check(t.transition, t.next_state, t.msg, t.non_possible_states)
	
	def test_wake_up_sequence_intent_not_recognized(self):
		"""
		Test a sequence simulating from a wake up to a non recognized event
		Try also to call all the other non possible state as test
		"""
		print(test_case_str('*** Start test_wake_up_sequence_intent_not_recognized ***'))
		
		# Remember to call at the begin of every test
		self._new_session()
		test_sequence = [TestData('wake_up', 'WaitingRhasspySession', WAKE_UP_MSG_1, FROM_WAIT_RH_NON_POSSIBLE),         # Wake up
						 TestData('session_started', 'SessionActive', SESS_STARTED_MSG_1, FROM_SESS_ACT_NON_POSSIBLE),   # simulate a session started reicived
						 TestData('asr_started', 'WaitingSpeech', START_LISTENING_MSG_1, FROM_WAIT_SPCH_NON_POSSIBLE),   # Going in Waiting Speech
						 TestData('text_captured', 'WaitingIntent', TEXT_CAPTURED_MSG_1, FROM_WAIT_INTENT_NON_POSSIBLE), # Process Intent
						 TestData('intent_not_recognized', 'SessionActive', INTENT_NOT_RECOGN_MSG_1, FROM_SESS_ACT_NON_POSSIBLE),# simulate intent not recognized
						 TestData('rh_session_reset', 'WaitingRhasspySession', RH_RESET_SESS_MSG_1, FROM_WAIT_RH_NON_POSSIBLE), # simulate a closing session
						]
		for t in test_sequence:
			self._call_transition_full_check(t.transition, t.next_state, t.msg, t.non_possible_states)

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


		test_sequence = [TestData('session_started', 'SessionActive', SESS_STARTED_MSG_1, FROM_SESS_ACT_NON_POSSIBLE),   # simulate a session started reicived
						 TestData('asr_started', 'WaitingSpeech', START_LISTENING_MSG_1, FROM_WAIT_SPCH_NON_POSSIBLE),   # Going in Waiting Speech
						 TestData('text_captured', 'WaitingIntent', TEXT_CAPTURED_MSG_1, FROM_WAIT_INTENT_NON_POSSIBLE), # Process Intent
						 TestData('intent_not_recognized', 'SessionActive', INTENT_NOT_RECOGN_MSG_1, FROM_SESS_ACT_NON_POSSIBLE),# simulate intent not recognized
						 TestData('rh_session_reset', 'WaitingRhasspySession', RH_RESET_SESS_MSG_1, FROM_WAIT_RH_NON_POSSIBLE), # simulate a closing session
						]
		
		# Try to call wake after every state in test_sequence
		done = []
		for t in test_sequence:
			self._call_transition_full_check(t.transition, t.next_state, t.msg, t.non_possible_states)
			self._call_transition(wake_up_intp.transition, wake_up_intp.next_state, wake_up_intp.msg)
			# Restore the point
			done.append(t)
			for d in done:
				self._call_transition(d.transition, d.next_state, d.msg)
		
		#	self._call_transition_full_check(t.transition, t.next_state, t.msg, t.non_possible_states)
	
	def test_close_session(self):
		"""
		Try to close the session from every designed state
		"""
		print(test_case_str('*** Start test_close_session ***'))
		# Remember to call at the begin of every test
		self._new_session()
		# Wake up, we want to move to this transition
		self._call_transition('wake_up', 'WaitingRhasspySession', WAKE_UP_MSG_1)

		close_sess_intp = TestData('rh_session_reset', 'WaitingRhasspySession', RH_RESET_SESS_MSG_1, FROM_WAIT_RH_NON_POSSIBLE) # simulate a closing session
		test_sequence = [TestData('session_started', 'SessionActive', SESS_STARTED_MSG_1, FROM_SESS_ACT_NON_POSSIBLE),   # simulate a session started reicived
						 TestData('asr_started', 'WaitingSpeech', START_LISTENING_MSG_1, FROM_WAIT_SPCH_NON_POSSIBLE),   # Going in Waiting Speech
						 TestData('text_captured', 'WaitingIntent', TEXT_CAPTURED_MSG_1, FROM_WAIT_INTENT_NON_POSSIBLE), # Process Intent
						 TestData('intent_not_recognized', 'SessionActive', INTENT_NOT_RECOGN_MSG_1, FROM_SESS_ACT_NON_POSSIBLE),# simulate intent not recognized				 
						]
		
		# Try to call wake after every state in test_sequence
		done = []
		for t in test_sequence:
			self._call_transition_full_check(t.transition, t.next_state, t.msg, t.non_possible_states)
			self._call_transition(close_sess_intp.transition, close_sess_intp.next_state, close_sess_intp.msg)
			# Restore the point
			done.append(t)
			for d in done:
				self._call_transition(d.transition, d.next_state, d.msg)
		
		#	self._call_transition_full_check(t.transition, t.next_state, t.msg, t.non_possible_states)
	

class TestStateMachineExternalWakeUp(unittest.TestCase):

	@classmethod
	def setUp(self):
		pass

	@classmethod
	def tearDownClass(cls):
		pass
	def test_external_wake_sequence(self):
		session, dialogue_sm = get_session()
		
		session.external_request()
		with self.assertRaises(MachineError) as cm:
			session.wake_up()
		session.session_started()


if __name__ == '__main__':
	import rosunit
	rosunit.unitrun("python_test", 'test_sequence', TestStateMachineWakeUp)
	#rosunit.unitrun("python_test", 'test_sequence', TestStateMachineExternalWakeUp)