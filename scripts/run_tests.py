#!/usr/bin/env python3

import unittest
import rosunit
import sys

# TEST import
from lisa_mqtt.dialogue import DialogueSession

from lisa_mqtt.tests.ut_sm_test import TestLisaStateMachineWakeUpWord, TestLisaStateMachineExternalRequest, TestLisaStateMachineMixRequests
from lisa_mqtt.tests.ros_test import TestLisaRosInteraction


if __name__ == '__main__':
	print(sys.argv)
	rosunit.unitrun("lisa_mqtt_ros_bridge", "test_state_machine_wake_up_word", TestLisaStateMachineWakeUpWord)
	rosunit.unitrun("lisa_mqtt_ros_bridge", "test_state_machine_external_request", TestLisaStateMachineExternalRequest)
	rosunit.unitrun("lisa_mqtt_ros_bridge", "test_state_machine_mix_request", TestLisaStateMachineMixRequests)
	rosunit.unitrun("lisa_mqtt_ros_bridge", "test_ros_interaction", TestLisaRosInteraction)
	