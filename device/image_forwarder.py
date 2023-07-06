#!/usr/bin/env python

import logging
import rospy
from sensor_msgs.msg import Image
from bridge import Bridge


class ImageForwarder(Bridge):
    def __init__(
        self,
        mqtt_topic,
        client_id="image_forwarder",
        packet_size=1024,
        user_id="",
        password="",
        host="localhost",
        port=1883,
        keepalive=60,
        qos=0,
        exit_on_complete=True,
        enable_logging=True,
    ):
        """
        Constructor method
        :param mqtt_topic: The topic to publish/subscribe to
        :param client_id: The ID of the client
        :param packet_size: The maximum size of each packet in bytes
        :param user_id: The user ID for the broker
        :param password: The password for the broker
        :param host: The hostname or IP address of the broker
        :param port: The port number of the broker
        :param keepalive: The keepalive interval for the client
        :param qos: The Quality of Service that determines the level of guarantee
        for message delivery between MQTT client and broker.
        """

        # Subscribe to the ROS topic
        self.sub = rospy.Subscriber("/PR_FE/feature_img", Image, self.image_callback)

        self.packet_size = packet_size
        self.num_packets_forwarded = 0
        self.exit_on_complete = exit_on_complete

        if enable_logging:
            self.logger = logging.getLogger(__name__)
            self.logger.setLevel(logging.INFO)
            handler = logging.StreamHandler()
            handler.setLevel(logging.INFO)
            formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)

        super().__init__(mqtt_topic, client_id, user_id, password, host, port, keepalive, qos)

    def image_callback(self, msg):
        try:
            # 1. Convert the ROS message to a bytearray
            byte_array = bytearray(msg.data)

            # 2. Split the byte array into smaller packets
            num_packets = (len(byte_array) + self.packet_size - 1) // self.packet_size
            for i in range(num_packets):
                start = i * self.packet_size
                end = (i + 1) * self.packet_size
                packet = byte_array[start:end]
                self.publish(self.mqtt_topic, message=packet)

                self.logger.info(
                    "Forwarded packet {} of {} with payload size {}".format(
                        self.num_packets_forwarded + i + 1, num_packets, len(packet)
                    )
                )

            # Increment the number of packets forwarded
            self.num_packets_forwarded += num_packets

            # Unsubscribe from the ROS topic if we've forwarded the desired number of packets
            if self.num_packets_forwarded >= self.num_packets:
                # Unsubscribe from the ROS topic
                self.sub.unregister()
                rospy.loginfo("Forwarded {} packets, unsubscribing from topic".format(self.num_packets))
                if self.exit_on_complete:
                    rospy.signal_shutdown("Image forwarding complete")

        except Exception as e:
            self.logger.error("Error occurs when forwarding image: {}".format(e))

    def run(self):
        # Spin the ROS node to receive messages
        rospy.spin()