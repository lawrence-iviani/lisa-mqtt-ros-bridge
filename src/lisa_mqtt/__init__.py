


from collections import OrderedDict, namedtuple
import time
current_milli_time = lambda: int(round(time.time() * 1000))
time_ref  = current_milli_time()

Topic = namedtuple('Topic', ['general_topic', 'specific_topic', 'function_handler'])

ROS_SERVICE_PREFIX = '/lisa/'

def _print_debug(msg):
	print("[{}ms]-{}".format(current_milli_time()-time_ref, msg))


class FixSizeOrderedDict(OrderedDict):
	# https://stackoverflow.com/questions/49274177/need-python-dictionary-to-act-like-deque-have-maximum-length
	def __init__(self, *args, max=0, **kwargs):
		self._max = max
		super().__init__(*args, **kwargs)

	def __setitem__(self, key, value):
		OrderedDict.__setitem__(self, key, value)
		if self._max > 0:
			if len(self) > self._max:
				self.popitem(False)


def _print_topic_data(handle_name, specific_topic, payload, parameters):
	_topic_msg = "\n\tparameters: " + "|".join(["{}={}".format(t, payload[t]) for t in parameters]) if parameters is not None else ""
	_print_debug('{}->{} payload:{} {}'.format(handle_name, specific_topic, payload, _topic_msg))
	
	
def _print_status(function, session, enter=True):
	if enter:
		msg = 'entering in'
	else:
		msg = 'exiting from'
	_print_debug('%%%%% {} {}, with status {} '.format(msg, function, session.state))


class ManagerInterface:
	"""
	A virtual interface 
	"""
	
	def __init__(self, mqtt_client, args):
		self._mqtt_client = mqtt_client
	
	def init_ros_topics(self):
		"""
		Must be called from outside, this init all the ROS calls (topic, service, actionlib....)
		"""
	
		# services
		# ---	
		# actionlib
		# ---	
		# topics to be published
		return NotImplementedError()
	
	@property
	def mqtt_topics_to_subscribe(self):
		return NotImplementedError()
		
	