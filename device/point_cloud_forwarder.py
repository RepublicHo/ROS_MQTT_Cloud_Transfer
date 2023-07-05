#!/usr/bin/env python

from bridge import Bridge
import rospy
import struct
import numpy as np
from sensor_msgs.msg import PointCloud
import logging


class PointCloudForwarder(Bridge):

    def __init__(self, mqtt_topic, client_id = "pc_forwarder", 
                 num_point_clouds = 100, user_id="", password="", 
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
        rospy.init_node('point_cloud_forwarder', anonymous=True)
        
        # Subscribe to the ROS topic
        self.sub = rospy.Subscriber('/PR_BE/point_cloud', PointCloud, self.pc_callback)
        
        self.num_point_clouds = num_point_clouds
        self.num_point_clouds_forwarded = 0
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
        
    def pc_callback(self, data):
        
        try:
            # Convert the PointCloud message to a list of points. 
            point_array = np.array([(p.x, p.y, p.z) for p in data.points])
            
            cloud_points = point_array.tolist()

            # Convert each tuple in the list of points to a list of floats
            cloud_points_float = [[float(i) for i in point] for point in cloud_points]

            # Flatten the list of lists into a single list of floats
            cloud_points_flat = [coord for point in cloud_points_float for coord in point]

            # Pack the list of floats into a binary string
            binary_msg = struct.pack('<%sf' % len(cloud_points_flat), *cloud_points_flat)

            # Publish the binary message to the MQTT topic
            self.publish(binary_msg)
            self.logger.info("Forwarded image {} with payload size {}".format(self.num_point_clouds_forwarded, len(binary_msg)))
            # Increment the number of images forwarded
            self.num_point_clouds_forwarded += 1
            
            # Unsubscribe from ROS topic if we have forwarded the desired number of point clouds
            if self.num_point_clouds_forwarded >= self.num_point_clouds:
                self.sub.unsubscribe()
                rospy.loginfo("Forwarded {} point clouds, unsubscribing from topic".format(self.num_images))
                if self.exit_on_complete:
                    rospy.signal_shutdown("Point Cloud forwarding complete")
            
        except Exception as e:
            self.logger.error("Error occurs when forwarding point cloud: {}".format(e))

    def run(self):
        # Spin the ROS node to receive messages
        rospy.spin()