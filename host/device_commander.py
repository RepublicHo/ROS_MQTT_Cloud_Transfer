import paho.mqtt.client as mqtt
import base64
import struct
import rospy
import os
import config as CONFIG
import iot_status_checker as status_checker

from bridge import Bridge


class DeviceCommander(Bridge):
    
    def __init__(self, client_id = "commander", 
                 user_id="", password="", 
                 host="localhost", port="1883", keepalive=60, qos=0):
        """
        Constructor method for the ToMqttBridge class
        :param client_id: The ID of the client
        :param user_id: The user ID for the broker
        :param password: The password for the broker
        :param host: The hostname or IP address of the broker
        :param port: The port number of the broker
        :param keepalive: The keepalive interval for the client
        :param qos: The Quality of Service that determines the level of guarantee 
        for message delivery between MQTT client and broker. 
        """
        
        self.COMMAND_TOPIC = "command"
        self.RESPONSE_TOPIC = "command_response"
        
        self.DATA_TOPISCS = {"point_cloud": "/data/point_cloud",
                             "image": "/data/img"}
        
        super().__init__(client_id, user_id, 
                         password, host, port, keepalive, qos)
    
    def on_connect(self, client, userdata, flags, rc):
        """
        Callback function called when the client successfully connects to the broker
        """
        print(f"Connected to MQTT broker with result code {str(rc)}")
        self.client.subscribe(self.RESPONSE_TOPIC)
        self.client.subscribe(self.DATA_TOPISCS["cloud_point"])
        self.client.subscribe(self.DATA_TOPISCS["image"])
        self.timeout = 0
    
    # Define the function to handle incoming messages
    def on_message(self, client, userdata, msg):
        # Decode the base64-encoded message payload
        data = base64.b64decode(msg.payload)
        print("msg: " + msg)
        print("data: " + data)
        print("Commander received message on topic " + msg.topic + " with payload size " + str(len(msg.payload)))
        
        # Handle the data based on the topic and its type
        if msg.topic == self.DATA_TOPISCS["cloud_point"]:
            print("->point cloud received!")
            self.process_pointcloud(msg)
        elif msg.topic == self.DATA_TOPISCS["image"]:
            print("->image received!")
            self.process_image(data)
        else:
            print("->Exception occurs!")
    
    def process_pointcloud(self, msg):
        # Unpack the binary message into a list of floats
        cloud_points_flat = struct.unpack('<%sf' % (len(msg.payload) // 4), msg.payload)

        # Convert the list of floats to a list of tuples representing points
        cloud_points = [(cloud_points_flat[i], cloud_points_flat[i+1], cloud_points_flat[i+2]) for i in range(0, len(cloud_points_flat), 3)]
        print("todo...")
    
    def process_image(self, msg):
        print("todo...")
    
    def send_commands(self):
        return 1

    def check_device_power_status(self):
        
        status = status_checker.get_device_status()
        
        if status:
            print("::: Device is ON")
            return 1
        else:
            print("::: Device is OFF, please check the device")
            return 0
    
    # Define the function to check if the capturing algorithm is on
    def check_algorithm_status(self):
        self.publish()
    
    # Define the function to enable the vio capturing algorithm
    def enable_vio_algorithm(self):
        self.publish()
    
    def publish(self, topic, message, qos=0):
        """
        Publish a message to the MQTT broker
        :param message: The message to publish
        """
        self.client.publish(topic, message, qos) 
            
    def callbacks(self, data):
        return 0
    
    def msg_process(self, msg):
        pass
        
    
    # define clear
    def clear(self):
        if os.name == 'nt': 
            # If the operating system is Windows, then the function executes the cls command 
            # using the os.system() function to clear the terminal screen.
            os.system('cls')
            
        else:
        # If the operating system is not Windows (i.e., Mac or Linux), 
        # then the function executes the clear command instead.
            os.system('clear')
    
    def menu(self):

        self.clear()
        status = self.check_device_power_status()
        if status == 0:
        
            print(
                '____________________________\n'
                ':::', CONFIG.APP.APPLICATION, CONFIG.APP.VERSION, ':::\n'
                ':::   ', CONFIG.APP.AUTHOR, '  :::\n'
                '¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯'
            )
            print("::: INFO :::")
            print("Checking if the device is on, wait")
    
    
    # Define the function to initiate data transfer
    def start_data_transfer(self, data):
        self.publish()

def main():
    rospy.init_node('device_commander', anonymous=True)
    
    commander_bridge = DeviceCommander(host=CONFIG.CONNECTION.BROKER, port=1883, qos=2)
    
    commander_bridge.menu()

    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        """
        If ROS interrupt exception is raised (e.g., if the user
        presses Ctrl-C to stop the program), the exception is
        caught and ignored. 
        """
        pass