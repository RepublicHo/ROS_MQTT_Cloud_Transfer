#!/usr/bin/env python
# coding: utf-8

import yaml
import json

import rospy
import paho.mqtt.client as mqtt

from ros_mqtt_bridge.args_setters import ArgsSetters


class ROSToMQTT(ArgsSetters):

    def __init__(self, from_topic, to_topic, message_type):
        """
        A class that bridges ROS messages to MQTT messages.
        """
        super(ROSToMQTT, self).__init__(message_type)

        self.__mqtt_client = None

        # Set the MQTT topic to publish to and the ROS topic to subscribe to.
        self.args["mqtt"]["publish"]["topic"] = to_topic
        self.args["ros"]["wait_for_message"]["topic"] = from_topic
        self.args["ros"]["wait_for_message"]["topic_type"] = self.args["ros"]["data_class"]

    def connect_ros(self):
        """
        Connect to ROS by initializing a node.
        """
        if "name" not in self.args["ros"]["init_node"]:
            self.args["ros"]["init_node"]["name"] = "ros_mqtt_bridge"
            self.args["ros"]["init_node"]["anonymous"] = True
        rospy.init_node(**self.args["ros"]["init_node"])

    def connect_mqtt(self):
        """
        Connect to MQTT by creating a client and connecting to a broker.
        """
        self.__mqtt_client = mqtt.Client(**self.args["mqtt"]["client"])
        if self.args["mqtt"]["tls"] is not None:
            self.set_mqtt_tls()
        self.__mqtt_client.connect(**self.args["mqtt"]["connect"])

    def set_mqtt_tls(self):
        """
        Set up TLS for secure MQTT communication.
        """
        self.__mqtt_client.tls_set(**self.args["mqtt"]["tls"])
        self.__mqtt_client.tls_insecure_set(True)

    def start(self):
        """
        Start the ROS-to-MQTT bridge by subscribing to a ROS topic and publishing messages to an MQTT topic.
        """
        self.connect_mqtt()
        self.connect_ros()
        self.__rospy_rate = rospy.Rate(**self.args["ros"]["rate"])
        while not rospy.is_shutdown():
            try:
                # Wait for a message on the ROS topic and convert it to YAML format.
                message_yaml = str(rospy.wait_for_message(**self.args["ros"]["wait_for_message"]))
                # Convert the YAML message to JSON and set the MQTT payload.
                self.args["mqtt"]["publish"]["payload"] = json.dumps(yaml.load(message_yaml))
                # Publish the MQTT message.
                self.__mqtt_client.publish(**self.args["mqtt"]["publish"])
                # Wait for the next message.
                self.__rospy_rate.sleep()
            except rospy.ROSException:
                pass
            except rospy.ROSInterruptException:
                break
