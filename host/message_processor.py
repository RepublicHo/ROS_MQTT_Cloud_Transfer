#!/usr/bin/env python

from bridge import Bridge
import rospy
import struct
import logging
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud
import binascii
import datetime
import os

class ImageProcessor(Bridge):
    # Define class constants for magic numbers
    DEFAULT_PACKET_SIZE = 1024
    DEFAULT_QOS = 0
    DEFAULT_KEEPALIVE = 60
    DEFAULT_EXIT_ON_COMPLETE = True
    DEFAULT_ENABLE_LOGGING = True
    def __init__(
            self, 
            mqtt_topic, 
            client_id = "image_processor", 
            packet_size=DEFAULT_PACKET_SIZE,
            user_id="", 
            password="", 
            host="localhost", 
            port="1883", 
            keepalive=DEFAULT_KEEPALIVE, 
            qos=DEFAULT_QOS,
            exit_on_complete=DEFAULT_EXIT_ON_COMPLETE,
            enable_logging=DEFAULT_ENABLE_LOGGING
    ):
        
        self.received_packets = []
        
        # Validate user inputs
        if packet_size <= 0:
            raise ValueError("Packet size must be a positive integer")
        
        self.packet_size = packet_size
        self.num_packets_received = 0
        self.num_image_received = 0
        self.exit_on_complete = exit_on_complete

        # Set initial value of processing_enabled to False
        self.processing_enabled = False
        
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
            file_handler = logging.FileHandler("point_cloud_processor.log")
            file_handler.setLevel(logging.INFO)
            file_handler.setFormatter(formatter)
            self.logger.addHandler(file_handler)
            
        super().__init__(mqtt_topic, client_id, user_id, 
                         password, host, port, keepalive, qos)
    
    
    def on_connect(self, client, userdata, flags, rc): 
        """
        Callback function called when the client successfully connects to the broker
        """
        self.logger.info(f"Image Processor connected to MQTT broker with result code {str(rc)}")
        self.client.subscribe(self.mqtt_topic)
        self.timeout = 0
        
    def start_processing(self):
        """
        Enable image processing
        """
        self.processing_enabled = True
        self.logger.info("Image processing started")

    def stop_processing(self):
        """
        Disable image processing
        """
        self.processing_enabled = False
        self.logger.info("Image processing stopped")
        
    def on_message(self, client, userdata, msg):
       
       # If the message is from the topic targeted for the point cloud transmission, 
       # and the processing is enabled. 
       if msg.topic == self.mqtt_topic and self.processing_enabled: 
            try:
                self.logger.info(
                    "Received packet {} with payload size {}".format(
                        self.num_packets_received, len(msg)
                    )
                )
                # Append the received packet to the list of received packets
                self.received_packets.append(msg)
                # If we've received all of the packets for the image, reassemble the image
                if len(self.received_packets) == self.num_packets:
                    # Concatenate the received packets into a single byte array
                    byte_array = b"".join(self.received_packets)

                    # Create a new Image message and set its fields
                    image_msg = Image()
                    image_msg.header.stamp = rospy.Time.now()
                    image_msg.data = byte_array
                    self.logger.debug("An image is received.")
                    # Reset the list of received packets
                    self.received_packets = []
        
                    self.num_packets_received = 0
                    self.logger.info("Forwarded image {} successfully".format(self.num_image_received))
                    self.num_image_received += 1
                    
            except TypeError as e:
                self.logger.error("Type error occurs when processing Image: {}".format(e))
            
            except Exception as e:
                self.logger.error("Error occurs when processing Image: {}".format(e))
    
    

class PointCloudProcessor(Bridge):
    # Define class constants for magic numbers
    DEFAULT_QOS = 0
    DEFAULT_KEEPALIVE = 60
    DEFAULT_EXIT_ON_COMPLETE = True
    DEFAULT_ENABLE_LOGGING = True
    def __init__(
            self, 
            mqtt_topic, 
            client_id = "point_cloud_processor", 
            user_id="", 
            password="", 
            host="localhost", 
            port="1883", 
            keepalive=DEFAULT_KEEPALIVE, 
            qos=DEFAULT_QOS,
            exit_on_complete=DEFAULT_EXIT_ON_COMPLETE,
            enable_logging=DEFAULT_ENABLE_LOGGING
    ):
        self.num_point_clouds_received = 0
        self.exit_on_complete = exit_on_complete

        # Set initial value of processing_enabled to False
        self.processing_enabled = False
        
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
            file_handler = logging.FileHandler("point_cloud_processor.log")
            file_handler.setLevel(logging.INFO)
            file_handler.setFormatter(formatter)
            self.logger.addHandler(file_handler)
            
        super().__init__(mqtt_topic, client_id, user_id, 
                         password, host, port, keepalive, qos)
    
    def on_connect(self, client, userdata, flags, rc): 
        """
        Callback function called when the client successfully connects to the broker
        """
        self.logger.info(f"Point Cloud Processor connected to MQTT broker with result code {str(rc)}")
        self.client.subscribe(self.mqtt_topic)
        self.timeout = 0
        
    def on_message(self, client, userdata, msg):
       
       # If the message is from the topic targeted for the point cloud transmission, 
       # and the processing is enabled. 
       if msg.topic == self.mqtt_topic and self.processing_enabled: 
            try:
                self.logger.debug(
                    "Received point cloud {} with payload size {}".format(
                        self.num_point_clouds_received, len(msg)
                    )
                )
                # Increment the number of point clouds received. 
                self.num_point_clouds_received += 1
                
                # Decode the hexadecimal message to a binary string
                binary_msg = binascii.unhexlify(msg)

                # Unpack the binary string to a list of floats
                num_floats = len(binary_msg) // 4  # Each float is 4 bytes
                float_list = struct.unpack("<%sf" % num_floats, binary_msg)

                # Reshape the list of floats to a list of tuples representing the point cloud
                point_clouds = [(float_list[i], float_list[i+1], float_list[i+2]) for i in range(0, num_floats, 3)]
                
                # Save the cloud_points_flat data to a file with an index and the current date and time in the filename
                now = datetime.datetime.now()
                date_time_string = now.strftime("%Y-%m-%d_%H-%M-%S-%f")
                filename = f'cloud_sub_{date_time_string}.ply'
                foldername = 'point_cloud_sets'
                if not os.path.exists(foldername):
                    os.mkdir(foldername)
                    
                filepath = os.path.join(foldername, filename)
                with open(filepath, 'w') as f:
                    f.write(f'ply\nformat ascii 1.0\nelement vertex {len(point_clouds)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n')
                    for point in point_clouds:
                        f.write(f'{point[0]} {point[1]} {point[2]}\n')
                
            except TypeError as e:
                self.logger.error("Type error occurs when processing point cloud: {}".format(e))
            
            except Exception as e:
                self.logger.error("Error occurs when processing point cloud: {}".format(e))
        
    def start_processing(self):
        """
        Enable point cloud processing
        """
        self.processing_enabled = True
        self.logger.info("Point cloud processing started")

    def stop_processing(self):
        """
        Disable point cloud processing
        """
        self.processing_enabled = False
        self.logger.info("Point cloud processing stopped")
    
    # please use a single rospy.spin() loop that listens for messages on 
    # all of the relevant MQTT topics.    
    # def run(self):
    #     rospy.spin()
        

    