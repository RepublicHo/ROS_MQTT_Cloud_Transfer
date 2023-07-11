#!/usr/bin/env python

import logging
import rospy
from sensor_msgs.msg import Image
from bridge import Bridge


class ImageForwarder(Bridge):
    # Define class constants for magic numbers
    DEFAULT_PACKET_SIZE = 1024
    DEFAULT_QOS = 0
    DEFAULT_KEEPALIVE = 60
    DEFAULT_EXIT_ON_COMPLETE = True
    DEFAULT_ENABLE_LOGGING = True
    
    def __init__(
        self,
        mqtt_topic,
        client_id="image_forwarder",
        packet_size=DEFAULT_PACKET_SIZE,
        user_id="",
        password="",
        host="localhost",
        port=1883,
        keepalive=DEFAULT_KEEPALIVE,
        qos=DEFAULT_QOS,
        exit_on_complete=DEFAULT_EXIT_ON_COMPLETE,
        enable_logging=DEFAULT_ENABLE_LOGGING,
    ):
        
        self.is_forwarding = False
        
        # Validate user inputs
        if packet_size <= 0:
            raise ValueError("Packet size must be a positive integer")
        
        # Subscribe to the ROS topic
        self.sub = rospy.Subscriber("/PR_FE/feature_img", Image, self.image_callback)

        self.packet_size = packet_size
        self.num_packets_forwarded = 0
        self.num_image_forwarded = 0
        self.exit_on_complete = exit_on_complete

        if enable_logging:
            # Configure logging to both console and file
            self.logger = logging.getLogger(__name__)
            self.logger.setLevel(logging.INFO)
            formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")

            # Add console handler
            console_handler = logging.StreamHandler()
            console_handler.setLevel(logging.INFO)
            console_handler.setFormatter(formatter)
            self.logger.addHandler(console_handler)

            # Add file handler
            file_handler = logging.FileHandler("image_forwarder.log")
            file_handler.setLevel(logging.INFO)
            file_handler.setFormatter(formatter)
            self.logger.addHandler(file_handler)

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

            self.logger.info("Forwarded image {} successfully".format(self.num_image_forwarded))
            
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

    def start_forwarding(self):
        self.is_forwarding = True
        
    def stop_forwarding(self):
        self.is_forwarding = False
        self.sub.unregister()
        
    # def run(self):
    #     # Spin the ROS node to receive messages
    #     rospy.spin()