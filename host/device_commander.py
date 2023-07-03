import paho.mqtt.client as mqtt
import base64
import struct
import time
# import rospy
import threading
import os
import json
import config as CONFIG
import iot_status_checker as isc

from bridge import Bridge


class DeviceCommander(Bridge):
    
    def __init__(self, client_id = "commander", 
                 user_id="", password="", 
                 host="localhost", port=1883, keepalive=60, qos=0):
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

        
        self.DATA_TOPISCS = {"point_cloud": "/data/point_cloud",
                             "image": "/data/img"}

        self.lock = threading.Lock()
        
        # connected is used for checking if the host can connected to the MQTT broker
        self.connected = False
        
        self.status = 0 # 0 for not connected, 1 for connected, 2 for bug
        
        # constants for brokers
        self.COMMAND = "/iot_device/command"
        self.COMMAND_RESPONSE = "/iot_device/command_response"
        
        super().__init__(self.COMMAND_RESPONSE, client_id, user_id, 
                         password, host, port, keepalive, qos)
    
    def on_connect(self, client, userdata, flags, rc):
        """
        Callback function called when the client successfully connects to the broker
        """
        print(f"Connected to MQTT broker with result code {str(rc)}")
        
        # Set the connected (to MQTT broker) flag to true.
        self.connected = True
        # Subscribed to the command respomse topic
        self.subscribe(self.COMMAND_RESPONSE)
 
    # Define the function to handle incoming messages
    def on_message(self, client, userdata, msg):
        # Decode the base64-encoded message payload
        data = base64.b64decode(msg.payload)
        # testing code
        # print("msg: " + msg)
        # print("data: " + data)
        print("Commander received message on topic " + msg.topic + " with payload size " + str(len(msg.payload)))
        
        with self.lock:
            if self.status == 1 and msg.topic == self.COMMAND_RESPONSE:
                self.msg_process(msg)
    
    
    def msg_process(self, msg):
        
        # Decode the message payload from JSON format
        json_payload = json.loads(msg.payload.decode())
        print("Received message: ", json_payload)
        # Handle the data based on the topic and its type
        # if msg.topic == self.DATA_TOPISCS[""]:
        #     pass
        #     print("->point cloud received!")
        #     self.process_pointcloud(msg)
        # elif msg.topic == self.DATA_TOPISCS["image"]:
        #     print("->image received!")
        #     self.process_image(data)
        # else:
        #     print("->Exception occurs!")
      
        
    def process_pointcloud(self, msg):
        """
        Function to process the point cloud data received from the IoT device
        """
        # Unpack the binary message into a list of floats
        point_clouds_flat = struct.unpack('<%sf' % (len(msg.payload) // 4), msg.payload)

        # Convert the list of floats to a list of tuples representing points
        point_clouds = [(point_clouds_flat[i], point_clouds_flat[i+1], point_clouds_flat[i+2]) for i in range(0, len(point_clouds_flat), 3)]
        print("todo...")
    
    def process_image(self, msg):
        """
        Function to process the image data received from the IoT device
        """
        pass
    
    def send_commands(self, value):
        """
        Function to send commands via MQTT to request the IoT device to transfer image/point cloud data
        """ 
        
        if value == 1:
            self.publish(self.COMMAND, "image")
        elif value == 2:
            self.publish(self.COMMAND, "point_cloud")
        

    def check_device_power_status(self):
        
        status_checker = isc.StatusChecker(client_id = "status_checker", host="43.133.159.102", port=1883)
        
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
        When connected, publish a message to the MQTT broker
        :param message: The message to publish
        """
        if self.connected:
            self.client.publish(topic, message, qos) 
        else:
            print("Oops, no connection currently!!!")
            
    def callbacks(self, data):
        return 0

        

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
        self.status = self.check_device_power_status()
        self.clear()
        if self.status == 1:
            
            # network_thread = threading.Thread(self.)
            print(
                '____________________________\n'
                ':::', CONFIG.APP.APPLICATION, CONFIG.APP.VERSION, ':::\n'
                ':::   ', CONFIG.APP.AUTHOR, '  :::\n'
                '¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯'
            )
            print("::: INFO :::")
            
            # Define the menu options
            menu_options = [
                {"name": "Image", "value": 1},
                {"name": "Point Cloud", "value": 2},
                {"name": "Exit", "value": 0},
            ]
            self.client.loop_start()
            
            time.sleep(20)
            self.client.loop_stop()
            #   Display the menu and prompt the user for input
            # while True:
            #     # Display the menu options
            #     print("Menu:")
            #     for option in menu_options:
            #         print(f"{option['value']}. {option['name']}")
                
            #     # Prompt the user for input
            #     choice = input("Enter your choice: ")
                
            #     # Handle the user's choice
            #     if choice == "0":
            #         print("Exiting...")
            #         break
            #     elif choice.isdigit() and int(choice) in [option["value"] for option in menu_options]:
            #         chosen_option = [option for option in menu_options if option["value"] == int(choice)][0]
            #         print(f"You chose {chosen_option['name']}.")
                    
            #     else:
            #         print("Invalid choice. Please try again.")
    
    
    # Define the function to initiate data transfer
    def start_data_transfer(self, data):
        self.publish()

def main():
    # rospy.init_node('device_commander', anonymous=True)
    
    commander_bridge = DeviceCommander(host=CONFIG.CONNECTION.BROKER, port=1883, qos=2)
    
    commander_bridge.menu()

    
if __name__ == '__main__':
    # try:
    main()
    # except rospy.ROSInterruptException:
    #     """
    #     If ROS interrupt exception is raised (e.g., if the user
    #     presses Ctrl-C to stop the program), the exception is
    #     caught and ignored. 
    #     """
    #     pass