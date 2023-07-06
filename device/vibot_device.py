import paho.mqtt.client as mqtt
import threading
import json
import requests
import time
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import Image
from point_cloud_forwarder import PointCloudForwarder
from image_forwarder import ImageForwarder
import logging

class Vibot:
    def __init__(self, status_check_topic = "/iot_device/status_check", command_topic = "/iot_device/command",
                 client_id="device", user_id="", password="", 
                 host="43.133.159.102", port=1883, keepalive=60, qos=0):
       
        self.status_check_topic = status_check_topic
        self.command_topic = command_topic
        self.client_id = client_id
        self.user_id = user_id
        self.password = password
        self.host = host
        self.port = port
        self.keepalive = keepalive
        self.qos = qos
        
        self.disconnect_flag = False
        self.rc = 1
        self.timeout = 0

        # Create the MQTT client object, set the username and password, and register the callback functions
        self.client = mqtt.Client(self.client_id, clean_session=True)
        self.client.username_pw_set(self.user_id, self.password)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message

        # Connect to the broker
        self.connect()

    def connect(self):
        """
        Connect to the MQTT broker
        """
        while self.rc != 0:
            try:
                self.rc = self.client.connect(self.host, self.port, self.keepalive)
            except Exception as e:
                # If the connection fails, wait for 2 seconds before trying again
                # If the connection fails, log the error and wait for some time before trying again.
                logging.error(f"Failed to connect to MQTT broker: {e}")
                logging.info("Waiting for 2 seconds before retrying...")
                time.sleep(2)
                self.timeout += 2
                
                # Print some suggestions for potential soluations
                print("---\nSuggestions: ")
                print("1. Check the WIFI connection. Make sure the port num in the code is an integer instead of a string. ")
                print("2. Check the MQTT version in the cloud. You might encounter local loopback monitoring issue in mosquitto 2 and higher. (I encountered it in Aliyun). "
                      +"\n You may downgrade MQTT to 1.6 stable or configure mosquitto.conf as appropriate.")
                print(f"3. Check MQTT return code(rc), which currently is {self.rc} \n---")

    def disconnect(self):
        """
        Disconnect from the MQTT broker
        """
        logging.info("Disconnecting from MQTT broker...")
        self.disconnect_flag = True
        self.client.disconnect()
        logging.info("Disconnected from MQTT broker...")
    

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
        img_bridge = ImageForwarder(mqtt_topic="/data/img", host="43.133.159.102", port=1883, qos=2)
        img_bridge.run()
        
        
    def msg_process(self, msg):
        """
        Process incoming MQTT messages
        """
        print("processing message")
        msg = str(msg.payload.decode())
        if msg == "status_check":
            self.publish("/iot_device/status_response", "status_ok")
            print("sent to /iot_device/status_response")
        
        elif msg == "enable_vio_service":
            
            url = 'http://localhost:8000/Smart/algorithmEnable'

            # Define the data to update the resource with
            data = {'name': 'John Smith', 'age': 35}

            # Encode the data as JSON
            # json_data = json.dumps(data)

            # Define the headers for the request
            # headers = {'Content-type': 'application/json'}

            # Make the HTTP PUT request
            response = requests.put(url)
            # print(response)
            # Check the response status code
            if response.status_code == 200:
                    
                    print('Vio algorithm enabled\n')
            else:
                    print('Failed to enable vio algorithm, please enable it manually\n')
            
            message = {'type': 'enable_vio', 'code': response.status_code}
            json_message = json.dumps(message)
            self.publish("/iot_device/command_response", json_message)
        
        elif msg == "disable_vio_service":
            
            url = 'http://localhost:8000/Smart/algorithmDisable'

            response = requests.put(url)

            if response.status_code == 200:
                    print('Vio algorithm disabled\n')
            else:
                    print('Failed to enable vio algorithm, please disable it manually\n')
            
            message = {'type': 'disable_vio', 'code': response.status_code}
            json_message = json.dumps(message)
            self.publish("/iot_device/command_response", json_message)
                        
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
    sn = Vibot()
    sn.looping()
    
    # Continuously publish device status every 5 seconds

    # Stop MQTT client loop and disconnect from broker
    

