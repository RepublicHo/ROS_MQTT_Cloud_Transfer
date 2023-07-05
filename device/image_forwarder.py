#!/usr/bin/env python

from bridge import Bridge
import rospy
from sensor_msgs.msg import Image
import logging

class ImageForwarder(Bridge):
    
    def __init__(self, mqtt_topic, client_id = "pc_forwarder", 
                 num_images = 3, user_id="", password="", 
                 host="localhost", port="1883", keepalive=60, qos=0, 
                 exit_on_complete = True, enable_logging=True):
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
        self.exit_on_complete = exit_on_complete
        
        if enable_logging:
            self.logger = logging.getLogger(__name__)
            self.logger.setLevel(logging.INFO)
            handler = logging.StreamHandler()
            handler.setLevel(logging.INFO)
            formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)
        
        super().__init__(mqtt_topic, client_id, user_id, 
                         password, host, port, keepalive, qos)
        
    def image_callback(self, msg):
        
        try:
            # Convert the ROS message to a bytearray
            byte_array = bytearray(msg.data)
            
            self.publish(message=byte_array)
    
            self.logger.info("Forwarded image {} with payload size {}".format(self.num_images_forwarded, len(byte_array)))
            # Increment the number of images forwarded
            self.num_images_forwarded += 1
            
            # Unsubscribe from the ROS topic if we've forwarded the desired number of images
            if self.num_images_forwarded >= self.num_images:
                # Unsubscribe from the ROS topic
                self.sub.unregister()
                rospy.loginfo("Forwarded {} images, unsubscribing from topic".format(self.num_images))
                if self.exit_on_complete:
                    rospy.signal_shutdown("Image forwarding complete")
                    
        except Exception as e:
            self.logger.error("Error occurs when forwarding image: {}".format(e))
        
    def run(self):
        # Spin the ROS node to receive messages
        rospy.spin()
