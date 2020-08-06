#!/usr/bin/env python3

import threading

## For ROS

# Intent Detection
import rospy
from std_msgs.msg import String

# Talk Service
import rospy
import actionlib

# For MQTT
import json
import argparse

# pip install paho-mqtt
import paho.mqtt.client as mqtt

# Intent Detection
from lisa_interaction_msgs.srv import IntentService
from lisa_interaction_msgs.msg import IntentMessage

# TTS actionlib messages
from lisa_interaction_msgs.msg import LisaUtterAction, LisaUtterFeedback, LisaUtterResult

MQTT_PORT = 12183 # MQTT Broker Port
USE_INTENT_SERVICE = True
ROS_SERVICE_PREFIX = '/lisa/'


# ROS global topics
pub_localized_source = rospy.Publisher(ROS_SERVICE_PREFIX + 'source/ssl', String, queue_size=10)
pub_tracked_source = rospy.Publisher(ROS_SERVICE_PREFIX + 'source/sst', String, queue_size=10)
pub_intent = None
pub_intent_not_rec = None
ros_utterance_server = None


def init_intention_topics():
    rospy.loginfo("Declaring ROS topics for intent recognition")
    pub_intent = rospy.Publisher(ROS_SERVICE_PREFIX + 'intent', IntentMessage, queue_size=10)
    pub_intent_not_rec = rospy.Publisher(ROS_SERVICE_PREFIX + 'intent/not_recognized', String, queue_size=10)
    rospy.loginfo("pub_intent: {} ---- pub_intent_not_rec:{}".format(pub_intent,pub_intent_not_rec))
    return pub_intent, pub_intent_not_rec


# MQTT FUNCTIONS
def on_connect(client, userdata, flags, rc):
    rospy.loginfo("Subscribing MQTT topics for intent recognition")
    """Called when connected to MQTT broker."""
    #client.subscribe("hermes/#")
    #  intentnion listening
    client.subscribe("hermes/intent/#")
    client.subscribe("hermes/nlu/intentNotRecognized")

    # source listening
    client.subscribe("lisa/ssl/source")
    client.subscribe("lisa/sst/source")

    # end action for tts
    client.subscribe("hermes/tts/sayFinished")
    client.subscribe("hermes/error/tts")
    # for progress
    # client.subscribe("hermes/audioServer/default/playBytes/#")

# [DEBUG:2020-07-19 23:00:13,138] rhasspyserver_hermes: Subscribed to hermes/tts/sayFinished
# [DEBUG:2020-07-19 23:00:13,140] rhasspyserver_hermes: Subscribed to hermes/error/tts
# [DEBUG:2020-07-19 23:00:13,141] rhasspyserver_hermes: Subscribed to hermes/audioServer/default/playBytes/#


    rospy.loginfo("Connected. Waiting for subscribed MQTT topics.")("Connected. Waiting for subscribed MQTT topics.")


def on_disconnect(client, userdata, flags, rc):
    """Called when disconnected from MQTT broker."""
    client.reconnect()


def on_message(client, userdata, msg):
    """Called each time a message is received on a subscribed topic."""

    if msg.topic == "lisa/ssl/source" or msg.topic == "lisa/sst/source":
        if msg.topic == "lisa/ssl/source":
            #type_src = "LOC"
            pub_localized_source.publish(str(msg))
        elif msg.topic == "lisa/sst/source":
            #type_src = "TRACK"
            pub_tracked_source.publish(str(msg))
        else:
            pass
    elif msg.topic == "hermes/nlu/intentNotRecognized":
        intent_str = "Unrecognized command!"
        service_name = ROS_SERVICE_PREFIX + 'intent/not_recognized'
        rospy.loginfo("Unrecognized command!")
        try:
            if USE_INTENT_SERVICE:
                # rospy.loginfo("Calling service {}".format(service_name))
                intent = rospy.ServiceProxy(service_name, IntentService)
                resp = intent(intent_str, "", 0.0)
                # rospy.loginfo("Result: intent>{}< - resp:>{}<".format(intent, resp))
            else:
                pub_intent_not_rec.publish("Unrecognized command!")
        except Exception as e:
            rospy.logerr("Error calling/publishing  {}}: {}".format(service_name, e))
    elif "hermes/intent/" in msg.topic:
        # decode the message
        nlu_payload = json.loads(msg.payload)

        # get some info about the incoming topic
        topic = msg.topic
        intent_str = nlu_payload["intent"]
        input_str = nlu_payload["input"]
        slots = ["{}={}".format(slot['slotName'], slot['rawValue']) for slot in nlu_payload['slots']]
        site_id = nlu_payload["siteId"]
        rospy.loginfo("Received MQTT topic: {} - slots: {} - original str: {} - original intent {}".
                      format(topic, slots, input_str, intent_str))

        # prepare for return message, this is how we map the MQTT intent in a ros message
        # string intent_name
        # string[] pay_load
        # float32 confidence
        # TODO: add field -> string stt_input
        ros_intent_name = intent_str['intentName'] if 'intentName' in intent_str else topic[len("hermes/intent/"):]
        ros_pay_load = slots
        ros_confidence = intent_str['confidenceScore'] if 'confidenceScore' in intent_str else 0.0
        rospy.loginfo("Coded as ROS message: {} - payload: {} - confidence: {} ".
                      format(ros_intent_name, ros_pay_load, ros_confidence))

        if USE_INTENT_SERVICE:
            service_name = ROS_SERVICE_PREFIX + 'intent'
            try:
                intent_srv = rospy.ServiceProxy(service_name, IntentService)
                resp = intent_srv(ros_intent_name, ros_pay_load, ros_confidence)
                rospy.loginfo("Response: " + str(resp))
            except Exception as e:
                rospy.logwarn("Error calling  {}: {}".format(service_name, e))
        else:
            try:
                msg = IntentMessage(intent_name=ros_intent_name, pay_load=ros_pay_load, confidence=ros_confidence)
                pub_intent.publish(msg)
            except Exception as e:
                rospy.logwarn("Error publishing intent topic  {}: {}".format(ros_intent_name, e))
        # client.publish("hermes/tts/say", json.dumps({"text": sentence, "siteId": site_id}))
    elif "hermes/tts/sayFinished" in msg.topic:
        rospy.loginfo("hermes/tts/sayFinished, telling the server")
        ros_utterance_server.say_finished()
    else:
        pass  # print(msg.topic)


def mqtt_client(args):
    rospy.loginfo("Starting MQTT Client")
    # Create MQTT client and connect to broker
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message

    client.connect(args.host, args.port)
    # start utter service listening on ROS and using MQTT client to call services
    ros_utterance_server = LisaUtterServer(client)

    try:
        client.loop_forever()
    except KeyboardInterrupt:
        pass
    finally:
        # _LOGGER.debug("Shutting down")
        rospy.loginfo("Shutting down MQTT Client")

# debug only
# def callback_recogn_intent(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#     #print(rospy.get_caller_id(), "\t\t +-+-+-+- INTENT: I heard --->>> ", data.data)
#
#
# def callback_not_recogn_intent(data):
#     rospy.loginfo(rospy.get_caller_id() + "Not recognized %s", data.data)
#     #print(rospy.get_caller_id(), "Not recognized: ", data.data)
#
#
# def callback_source(data):
#     pass
#     # rospy.loginfo(rospy.get_caller_id() + "Not recognized %s", data.data)
#     #print(rospy.get_caller_id(), "Source --->>> ", data.data)
########


from time import sleep
# from rhasspyhermes.tts import GetVoices, TtsError, TtsSay, TtsSayFinished


class LisaUtterServer:
        """
        An action lib for uttering sentence.
        TODO: improve with real feedback on utterance progress
        """

        def __init__(self, client):
                self.server = actionlib.SimpleActionServer('/lisa/say', LisaUtterAction, self.execute_callback, False)
                # self.server = actionlib.SimpleActionServer('do_dishes', DoDishesAction, self.execute_callback, False)
                self.server.start()
                self.mqtt_client = client

        def execute_callback(self, goal_handle):
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
                payload = json.dumps({"text": sentence})  # TtsSay(text=sentence, lang="en_EN")  # id=sentence for example?
                rospy.loginfo("Received sentence to utter: {}".format(payload))
                retval = self.mqtt_client.publish(topic="hermes/tts/say", payload=payload)
                rospy.loginfo("retval: {}".format(retval))
                # mimic listening for an action
                # TODO: check for real progress instead of estimated
                SLEEP_TIME = 0.082 # TODO: MAGIC NUMBER HERE!!!!
                for i in range(0, len(sentence)):
                    feedback_msg.percent_complete = float(i) / float(len(sentence))
                    # self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
                    self.server.publish_feedback(feedback_msg)
                    sleep(SLEEP_TIME)  # THE operation

                self.server.set_succeeded(result_msg)

        def say_finished(self):
            print('SAY FINSIHED HERE!!')



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
        help="An intent is called as service " + ROS_SERVICE_PREFIX + "intent_name, instead a topic is published at the address "
    )




    # start first the rospy node
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('lisa_mqtt_ros_bridge', anonymous=True)

    args = parser.parse_args()
    USE_INTENT_SERVICE = args.call_intent_service
    if USE_INTENT_SERVICE:
        rospy.loginfo('Calling service for intents')
    else:
        rospy.loginfo('Publish topics for intents')
        pub_intent, pub_intent_not_rec = init_intention_topics()


    # start MQTT listener client
    threading.Thread(target=mqtt_client, args=(args,), daemon=True).start()

    # keeps python from exiting until this node is stopped
    rospy.spin()






