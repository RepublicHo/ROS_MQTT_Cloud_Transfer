#!/usr/bin/env python

import paho.mqtt.client as mqtt
import threading
import json
import requests
import time
import rospy
import numpy as np
from bridge import Bridge
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import Image
from point_cloud_forwarder import PointCloudForwarder
from image_forwarder import ImageForwarder
import logging

class Vibot(Bridge):
    def __init__(
        self, 
        mqtt_topic,
        client_id="vibot_device", 
        user_id="", 
        password="", 
        host="43.133.159.102", 
        port=1883, 
        keepalive=60, 
        qos=0
    ):
       
        self.status_check_topic = "/iot_device/status_check"
        self.command_topic = "/iot_device/command"
        self.response_topic = "/iot_device/command_response"
        
        self.enable_vio_algorithm_url = 'http://localhost:8000/Smart/algorithmEnable'

        # Initialize the ROS forwarder node, which can
        # 1. Subscribe to a topic in ROS. 
        # 2. Immediately publish the message received to the MQTT topic in the cloud. 
        # Currently it can merely forward two types of message, 
        # as seen in point_cloud_forwarder.py and image_forwarder.py
        rospy.init_node("forwarder", anonymous=True)
        
        # Logging configuration
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)
        handler = logging.StreamHandler()
        handler.setLevel(logging.DEBUG)
        formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
        handler.setFormatter(formatter)
        self.logger.addHandler(handler)
        
        # We take the command topic as default mqtt topic. 
        super().__init__(self.command_topic, client_id, user_id, password, host, port, keepalive, qos)
    
    def point_cloud_transfer(self):
        
        # Create an instance of the forwarder class
        # QoS is set as 2 to ensure the message is delivered exactly once, which brings more overhead. 
        # TODO: you can specify how many point clouds to transfer 
        pc_bridge = PointCloudForwarder(mqtt_topic="/data/point_cloud", host="43.133.159.102", port=1883, qos=2)
        pc_bridge.run()
    
    def image_transfer(self):
        
        # Create an instance of the forwarder class
        # QoS is set as 2 to ensure the message is delivered exactly once, which brings more overhead.
        # TODO: you can specify how many images to transfer 
        img_bridge = ImageForwarder(mqtt_topic="/data/img", host="43.133.159.102", port=1883, qos=0)
        img_bridge.run()
        
        
    def msg_process(self, msg):
        """
        Process incoming MQTT messages from topic "/iot_device/command"
        """
        msg_topic = msg.topic
        msg = str(msg.payload.decode())
        self.logger.debug(f"Processing message {msg} from topic {msg_topic}")
        
        if msg == "status_check":
            # This message is used to verify the status of the IoT device. 
            # The current implementation involves the following steps:
            # 1. The host sends a string "status_check" to the IoT device via the MQTT broker.
            # 2. The device sends a string "status_ok" to the host via the broker to indicate that it is online and responsive.
            # 
            # TODO: To enhance the security of this verification process, additional authentication and authorization mechanisms 
            # could be implemented to ensure that only authorized hosts can send the "status_check" message, 
            # and only authorized devices can respond with the "status_ok" message.
            self.publish("/iot_device/status_response", "status_ok")
            self.logger.debug("Sent status_ok to /iot_device/status_response")
        
        elif msg == "enable_vio_service":

            # Make the HTTP PUT request
            http_response = requests.put(self.enable_vio_algorithm_url)

            # Check the response status code
            if http_response.status_code == 200: 
                self.logger.info('Vio algorithm enabled\n')
            else:
                self.logger.warning('Failed to enable vio algorithm, please enable it manually\n')
            
            message = {'type': 'enable_vio', 'code': http_response.status_code}
            json_message = json.dumps(message)
            self.publish(self.response_topic, json_message)
        
        elif msg == "disable_vio_service":
            
            url = 'http://localhost:8000/Smart/algorithmDisable'

            response = requests.put(url)

            if response.status_code == 200:
                    print('Vio algorithm disabled\n')
            else:
                    print('Failed to enable vio algorithm, please disable it manually\n')
            
            message = {'type': 'disable_vio', 'code': response.status_code}
            json_message = json.dumps(message)
            self.publish(self.response_topic, json_message)
                        
        elif msg == "point_cloud":
            message = {'type': 'point_cloud_transfer', 'code': 200}
            json_message = json.dumps(message)
            self.publish("/iot_device/command_response", message=json_message)
            print("--log--: sent to iot_device/command_response indicating we are starting the point cloud transfer")
            
            try:
                self.point_cloud_transfer()
            except rospy.ROSInterruptException:
                pass
            
        # elif msg == "end_point_cloud":
        #     print("End sending point cloud")
            
        elif msg == "image":
            message = {'type': 'image_transfer', 'code': 200}
            json_message = json.dumps(message)
            self.publish("iot_device/command_response", message=json_message)
            print("--log--: sent to iot_device/image indicating we are starting the image transfer")
            
            try:
                self.image_transfer()
            except rospy.ROSInterruptException:
                pass
            
        # elif msg == "end_image":
        #     print("End sending image")
            
        else:
            pass
        

    def looping(self, loop_timeout=30):
        """
        Start the MQTT client's event loop
        :param loop_timeout: The maximum time to wait for incoming messages before returning
        """
        # self.client.loop(loop_timeout)
        self.client.loop_forever()

    def on_connect(self, client, userdata, flags, rc):
        """
        Callback function called when the client successfully connects to the broker
        """
        print(f"Connected to MQTT broker with result code {str(rc)}")
        self.client.subscribe(self.status_check_topic)
        self.client.subscribe(self.command_topic)
        self.timeout = 0
        
        # Continuously publish device heartbeat 
        # daemon thread is running in the background and does not prevent the
        # main program from existing. when the main program exists, any 
        # remaining daemon threads are terminated automatically. 
        heartbeat_thread = threading.Thread(target=self.send_heartbeat, daemon=True)
        heartbeat_thread.start()
        
    
    def send_heartbeat(self):
        """
        Publish device heartbeat 
        """
        while True:
            self.publish("/iot_device/heartbeat", "heartbeat")
            logging.info("vibot: heartbeat sent")
            
            #testing code 
            # payload = {"name": "John", "age": 30, "city": "New York"}
            # json_payload = json.dumps(payload)
            # self.publish("/iot_device/command_response", json_payload)
            
            time.sleep(2)
        
    def on_disconnect(self, client, userdata, rc):
        """
        Callback function called when the client is unexpectedly disconnected from the broker
        """
        if rc != 0:
            if not self.disconnect_flag:
                logging.warning("Unexpected disconnection.")
                logging.warning("Trying reconnection")
                self.rc = rc
                self.connect()

    def on_message(self, client, userdata, msg):
        """
        Callback function called when an incoming message is received
        """
        print(f"iot device received a message from {msg.topic}")
        self.msg_process(msg)

    # def unsubscribe(self):
    #     """
    #     Unsubscribe from the MQTT topic
    #     """
    #     print("Unsubscribing")
    #     self.client.unsubscribe(self.status_check_topic)

    def disconnect(self):
        """
        Disconnect from the MQTT broker
        """
        print("Disconnecting")
        self.disconnect_flag = True
        self.client.disconnect()

    
    def publish(self, topic, message):
        """
        Publish a message to the MQTT broker
        :param message: The message to publish
        """
        self.client.publish(topic, message)

    def hook(self):
        """
        Gracefully shut down the MQTT client
        """
        # self.unsubscribe()
        self.disconnect()
        print("Shutting down")

    def get_timeout(self):
        """
        Get the amount of time elapsed since the connection attempt started
        """
        return self.timeout

if __name__ == "__main__":
    
    # Set up MQTT client and callbacks
    sn = Vibot(mqtt_topic=)
    sn.looping()
    

    

