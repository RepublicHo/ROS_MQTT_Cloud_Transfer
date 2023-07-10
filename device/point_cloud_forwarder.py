#!/usr/bin/env python

import logging
import rospy
import struct
import numpy as np
import binascii
from sensor_msgs.msg import PointCloud
from bridge import Bridge


class PointCloudForwarder(Bridge):
    # Define class constants for magic numbers
    DEFAULT_QOS = 0
    DEFAULT_KEEPALIVE = 60
    DEFAULT_EXIT_ON_COMPLETE = True
    DEFAULT_ENABLE_LOGGING = True
    
    def __init__(
        self,
        mqtt_topic,
        client_id="pc_forwarder",
        num_point_clouds=100,
        user_id="",
        password="",
        host="localhost",
        port=1883,
        keepalive=DEFAULT_KEEPALIVE,
        qos=DEFAULT_QOS,
        exit_on_complete=DEFAULT_EXIT_ON_COMPLETE,
        enable_logging=DEFAULT_ENABLE_LOGGING
    ):

        
        # Subscribe to the ROS topic
        self.sub = rospy.Subscriber("/PR_BE/point_cloud", PointCloud, self.pc_callback)

        self.num_point_clouds = num_point_clouds
        self.num_point_clouds_forwarded = 0
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
            file_handler = logging.FileHandler("point_cloud_forwarder.log")
            file_handler.setLevel(logging.INFO)
            file_handler.setFormatter(formatter)
            self.logger.addHandler(file_handler)

        super().__init__(mqtt_topic, client_id, user_id, password, host, port, keepalive, qos)

    def pc_callback(self, data):
        try:
            # 1. Convert the PointCloud message to a list of points.
            point_array = np.array([(p.x, p.y, p.z) for p in data.points])
            cloud_points = point_array.tolist()

            # 2. Convert each tuple in the list of points to a list of floats
            cloud_points_float = [[float(i) for i in point] for point in cloud_points]

            # 3. Flatten the list of lists into a single list of floats
            cloud_points_flat = [coord for point in cloud_points_float for coord in point]

            # 4. Pack the list of floats into a binary string
            binary_msg = struct.pack("<%sf" % len(cloud_points_flat), *cloud_points_flat)

            # 5. Convert the binary message to hexadecimal format
            hex_msg = binascii.hexlify(binary_msg).decode()

            # Publish the hexadecimal message to the MQTT topic
            self.publish(self.mqtt_topic, hex_msg)
            self.logger.info(
                "Forwarded point cloud {} with payload size {}".format(
                    self.num_point_clouds_forwarded, len(hex_msg)
                )
            )
            
            # Increment the number of point clouds forwarded
            self.num_point_clouds_forwarded += 1

            # Unsubscribe from ROS topic if we have forwarded the desired number of point clouds
            if self.num_point_clouds_forwarded >= self.num_point_clouds:
                # Unsubscribe from the ROS topic
                self.sub.unsubscribe()
                rospy.loginfo("Forwarded {} point clouds, unsubscribing from topic".format(self.num_images))
                if self.exit_on_complete:
                    rospy.signal_shutdown("Point Cloud forwarding complete")

        except Exception as e:
            self.logger.error("Error occurs when forwarding point cloud: {}".format(e))

    def run(self):
        # Spin the ROS node to receive messages
        rospy.spin()