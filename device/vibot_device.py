import paho.mqtt.client as mqtt
import threading
import json
import requests
import time
import subprocess
import re
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud
import struct

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
            except:
                # If the connection fails, wait for 2 seconds before trying again
                print("Connection failed, please check WiFi connection or MQTT broker in the cloud.")
                
            time.sleep(2)
            self.timeout += 2


    def disconnect(self):
        """
        Disconnect from the MQTT broker
        """
        print("Disconnecting")
        self.disconnect_flag = True
        self.client.disconnect()
        
    def callback(self, data):
        # global variables for testing
        global msg_index, num_index, sum

        # Convert the PointCloud2 message to a list of points. 
        point_array = np.array([(p.x, p.y, p.z) for p in data.points])
        cloud_points = point_array.tolist()

        # Convert each tuple in the list of points to a list of floats
        cloud_points_float = [[float(i) for i in point] for point in cloud_points]

        # Flatten the list of lists into a single list of floats
        cloud_points_flat = [coord for point in cloud_points_float for coord in point]

        # Test code: Save the binary message to a file with an index in the filename.
        filename = 'cloud_pub_%d.txt' % msg_index
        with open(filename, 'w') as f:
            for point in cloud_points_flat:
                sum += point
                f.write(str(num_index) + " " + str(point) + " " + str(sum) + '\n')
                num_index += 1

        # Increment the message index.
        msg_index += 1

        # Pack the list of floats into a binary string
        binary_msg = struct.pack('<%sf' % len(cloud_points_flat), *cloud_points_flat)

        # Publish the binary message to the MQTT topic
        self.publish("/data/point_cloud", binary_msg)

        rospy.loginfo("Forwarder forwards point cloud message with payload size %d " % len(binary_msg))    

    def point_cloud_transfer(self):
        rospy.init_node('point_cloud_forwarder', anonymous=True)

        # mqtt topic named ABC is for testing 
        # Create an instance of the ToMqttBridge class
        # QoS is set as 2 to ensure the message is delivered exactly once, which brings more overhead. 
        # bridge = ToMqttBridge(mqtt_topic="ABC", host="121.41.94.38", port=1883, qos=2)

        # Subscribe to the point cloud input topic and set the callback function
        rospy.Subscriber('/PR_BE/point_cloud', PointCloud, self.callback)

        # Start the MQTT client's event loop
        self.looping()
        
        rospy.spin()
        
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
            self.publish("iot_device/command_response", "")
            print("sent to iot_device/command_response")
            self.point_cloud_transfer()
            
        elif msg == "end_point_cloud":
            print("End sending point cloud")
            
        elif msg == "image":
            self.publish("/data/point_cloud", "image sent!")
            print("sent to iot_device/command_response")
            
        elif msg == "end_image":
            print("End sending image")
            
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
            print("vibot: heartbeat sent")
            
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
                print("Unexpected disconnection.")
                print("Trying reconnection")
                self.rc = rc
                self.connect()

    def on_message(self, client, userdata, msg):
        """
        Callback function called when an incoming message is received
        """
        print("iot device obtains the message in iot_device/command")
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
    

