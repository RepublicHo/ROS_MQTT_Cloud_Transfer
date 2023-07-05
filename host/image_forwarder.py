#!/usr/bin/env python

from bridge import Bridge
import rospy
from sensor_msgs.msg import Image
import requests

class ImageForwarder(Bridge):
    
    def __init__(self, mqtt_topic, client_id = "pc_forwarder", 
                 num_images = 10, user_id="", password="", 
                 host="localhost", port="1883", keepalive=60, qos=0):
        """
        Constructor method 
        :param mqtt_topic: The topic to publish/subscribe to
        :param client_id: The ID of the client
        :param user_id: The user ID for the broker
        :param password: The password for the broker
        :param host: The hostname or IP address of the broker
        :param port: The port number of the broker
        :param keepalive: The keepalive interval for the client
        :param qos: The Quality of Service that determines the level of guarantee 
        for message delivery between MQTT client and broker. 
        """
        
        # Initialize the ROS forwarder node, which can 
        # 1. Subscribe to a topic in ROS
        # 2. Immediately publish a point cloud message to the MQTT topic
        rospy.init_node('image_forwarder', anonymous=True)

        # Subscribe to the ROS topic
        self.sub = rospy.Subscriber('/PR_FE/feature_img', Image, self.callback)
        
        self.num_images = num_images
        self.num_images_forwarded = 0
        
        super().__init__(mqtt_topic, client_id, user_id, 
                         password, host, port, keepalive, qos)
        
    def image_callback(self, msg):
        
        # Convert the ROS message to a bytearray
        byte_array = bytearray(msg.data)
        
        self.publish(message=byte_array)
   
        rospy.loginfo("Forwarder forwards point cloud message with payload size %d " % len(byte_array))    
        # Increment the number of images forwarded
        self.num_images_forwarded += 1
        
        # Unsubscribe from the ROS topic if we've forwarded the desired number of images
        if self.num_images_forwarded >= self.num_images:
            # Unsubscribe from the ROS topic
            self.sub.unregister()
            rospy.loginfo("Forwarded {} images, unsubscribing from topic".format(self.num_images))
        
    def run(self):
        # Spin the ROS node to receive messages
        rospy.spin()
