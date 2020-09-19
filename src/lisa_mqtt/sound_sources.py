
from collections import OrderedDict
import json

# ROS Stuff
import rospy
from std_msgs.msg import String


from . import _print_debug, _print_topic_data, FixSizeOrderedDict, Topic, time_ref, current_milli_time, ROS_SERVICE_PREFIX, ManagerInterface


subscribe_topics = {'lisa/': 	['ssl/source', 'sst/source'],}
ros_topic_publish = {ROS_SERVICE_PREFIX + 'source/ssl': String,
					 ROS_SERVICE_PREFIX + 'source/sst': String,}


class SoundSourcesManager(ManagerInterface):
	
	pub_localized_source = None
	pub_tracked_source = None
	_ros_is_init = False

	def init_ros_topics(self):
		self.pub_localized_source = rospy.Publisher(ROS_SERVICE_PREFIX + 'source/ssl', String, queue_size=10)
		self.pub_tracked_source = rospy.Publisher(ROS_SERVICE_PREFIX + 'source/sst', String, queue_size=10)
		self._ros_is_init = True
	
	@property
	def mqtt_topics_to_subscribe(self):
		retval = OrderedDict()
		
		# print('subscribe_topics.items()=',subscribe_topics.items())
		for k,v  in subscribe_topics.items():
			# print(k,v)
			function_handler = self.handle_sound_source
			for t in v:
				general_topic = k
				specific_topic = t
				retval[k+t] = Topic(general_topic=k,specific_topic=t, function_handler=function_handler)	
		return retval
	
	def handle_sound_source(self, specific_topic, client, userdata, message):
		assert self._ros_is_init, "ROS node not init!" # add this check in all handlers that should use ros
		
		payload = json.loads(message.payload)
		if specific_topic == 'ssl/source':
			#_print_topic_data('ssl/source', specific_topic, payload, None)# ['siteId'])
			self.pub_localized_source.publish(str(message))
		elif specific_topic == 'sst/source':
			#_print_topic_data('sst/source', specific_topic, payload, None)
			self.pub_tracked_source.publish(str(message))
		else:
			_print_debug('handle_sound_source->{} +++UNHANDLED+++'.format(specific_topic))