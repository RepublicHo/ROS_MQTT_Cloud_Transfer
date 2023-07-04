import paho.mqtt.client as mqtt
import base64
import struct
import time
import sys
import queue
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
        
        self.user_input_queue = queue.Queue()
        
        self.status = 0 # 0 for not connected, 1 for connected, 2 for timeout or other bugs
        self.last_heartbeat_time = time.time()
        
        # constants for brokers
        self.DEVICE_HEARTBEAT = "/iot_device/heartbeat"
        self.COMMAND = "/iot_device/command"
        self.COMMAND_RESPONSE = "/iot_device/command_response"
        self.HEARTBEAT_TIMEOUT = 30 # seconds
        
        self.vio_enabled = False
        
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
        # data = base64.b64decode(msg.payload)
        # testing code
        # print("msg: " + msg)
        # print("data: " + data)
        
        # testing code 
        # print("Commander received message on topic " + msg.topic + " with payload size " + str(len(msg.payload)))
        
        if msg.topic == self.DEVICE_HEARTBEAT:
            self.last_heartbeat_time = time.time()
            # print("updated last_heartbeat", self.last_heartbeat_time)
            # print(1)
        elif msg.topic == self.DATA_TOPISCS["point_cloud"]:
            self.process_point_cloud(msg)
            
        elif self.status == 1:
            if msg.topic == self.COMMAND_RESPONSE:       
                self.msg_process(msg)
            else:
                print("Oops, timeout!")
    
    def is_json(self, string):
        try:
            json_object = json.loads(string)
        except ValueError:
            return False
        return True
    
    def msg_process(self, msg):
        
        if self.is_json(msg.payload.decode()):
            # Decode the message payload from JSON format
            json_msg = json.loads(msg.payload.decode())
            print("Received message: ", json_msg)
            
            if json_msg['type'] == "enable_vio" and json_msg['code'] == 200:
                self.vio_enabled = True
            elif json_msg['type'] == "disable_vio" and json_msg['code'] == 200:
                self.vio_enabled = False
            elif json_msg['type'] == "point_cloud_transfer" and json_msg['code'] == 200:
                self.subscribe()
        
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
      
        
    def process_point_cloud(self, msg):
        """
        Function to process the point cloud data received from the IoT device
        """
        print("Received message on topic " + msg.topic + " with payload size " + str(len(msg.payload)))
        
        # Unpack the binary message into a list of floats
        # point_clouds_flat = struct.unpack('<%sf' % (len(msg.payload) // 4), msg.payload)

        # # Convert the list of floats to a list of tuples representing points
        # point_clouds = [(point_clouds_flat[i], point_clouds_flat[i+1], point_clouds_flat[i+2]) for i in range(0, len(point_clouds_flat), 3)]
        # print("todo...")
    
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
        print("zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz")
        print(status)
        print("zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz")
        if status:
            print("::: Device is ON")
            return 1
        else:
            print("::: Device is OFF, please check the device")
            return 0
    
    def check_heartbeat(self):
        while True:
            # print("last heartbeat: ",  self.last_heartbeat_time, " time now: ", time.time())
            
            # Check if the heartbeat timeout has elapsed
            print("checked last_heartbeat", self.last_heartbeat_time)
            print(time.time() - self.last_heartbeat_time)
            if time.time() - self.last_heartbeat_time > self.HEARTBEAT_TIMEOUT:
                with self.lock:
                    self.status = 2 # timeout
                print("Heartbeat timeout (y for reconnect, n for stopping the program)")
                user_input = input("Enter your choice (Y/n): ").lower()
                
                if user_input == 'y':
                    self.menu()
                else:
                    self.client.loop_stop()
                    self.status = 0
                    sys.exit()
                    break
                
            time.sleep(1)
    
    # Define the function to check if the capturing algorithm is on
    def check_algorithm_status(self):
        self.publish()
    
    # Define the function to enable the vio capturing algorithm
    def enable_vio_algorithm(self):
        
        
        # Wait for 10 seconds and see if we can subscribe to message from topic RESPONSE
        
        self.publish(self.COMMAND, "enable_vio_service")
        time.sleep(3)
        if self.vio_enabled:
            print("Successfully enabled vio service")
        else:
            self.publish(self.COMMAND, "enable_vio_service")
            time.sleep(5)
            if self.vio_enabled:
                print("Successfully enabled vio service")
                self.publish(self.COMMAND, "im")
            else:
                print("Failed to enable vio service")
        
        
        
    # Define the function to disable the vio capturing algorithm
    def disable_vio_algorithm(self):
        
        
        # Wait for 10 seconds and see if we can subscribe to message from topic RESPONSE
        
        self.publish(self.COMMAND, "disable_vio_service")
        time.sleep(3)
        if not self.vio_enabled:
            print("Successfully disabled vio service")
        else:
            self.publish(self.COMMAND, "disable_vio_service")
            time.sleep(5)
            if not self.vio_enabled:
                print("Successfully disabled vio service")
            else:
                print("Failed to disable vio service")
        
    def prompt_user_input(self):
        choice = input("Enter your choice: ")
        self.user_input_queue.put(choice)
    
    def publish(self, topic, message, qos=0):
        """
        When connected, publish a message to the MQTT broker
        :param message: The message to publish
        """
        if self.connected:
            self.client.publish(topic, message, qos)
            print(f"published a message to topic {topic}") 
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
        with self.lock:
            self.status = self.check_device_power_status()
        print("=====================Status==================")
        print(self.status)
        print()
        # self.clear()
        
        if self.status == 1:
            # Continuously subscribe to heartbeat topic from IoT device. 
            self.subscribe(self.DEVICE_HEARTBEAT)
            self.client.loop_start()
            self.last_heartbeat_time = time.time()
            # heartbeat_thread = threading.Thread(target=self.check_heartbeat, daemon=True)
            # heartbeat_thread.start()
            time.sleep(1)
            # network_thread = threading.Thread(self.)
            print(
                '____________________________\n'
                ':::', CONFIG.APP.APPLICATION, CONFIG.APP.VERSION, ':::\n'
                ':::   ', CONFIG.APP.AUTHOR, '  :::\n'
                '¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯'
            )
            
            # Define the menu options
            menu_options = [
                {"name": "Enable Vio algorithm Service", "value": 1},
                {"name": "Disable Vio algorithm Service", "value": 2},
                {"name": "Obtain Point Cloud", "value": 3},
                {"name": "Obtain Image", "value": 4},
                {"name": "Exit", "value": 0}
            ]
                
                
            # Display the menu and prompt the user for input
            while self.status == 1:
            # Display the menu options
                print("\n::: Menu :::")
                for option in menu_options:
                    print(f"{option['value']}. {option['name']}")
                    
                # Prompt the user for input
                
                # user_input_thread = threading.Thread(target=self.prompt_user_input, daemon=True)
                # user_input_thread.start()
                
                # wait for user input or heartbeat timeout
                # while user_input_thread.is_alive():
                #     with self.lock:
                #         if self.status != 1:
                #             user_input_thread.join()
                #             break
                #     time.sleep(4)
                
                # if user_input_thread.is_alive():
                #     user_input_thread.join()
                    
                # try:
                #     choice = self.user_input_queue.get(block=False)
                # except queue.Empty:
                #     continue
                
                choice = input("Enter your choice: ")
                # Handle the user's choice
                if choice == "0":
                    print("Exiting...")
                    break
                elif choice == "1":
                    self.enable_vio_algorithm()
                elif choice == "2":
                    self.disable_vio_algorithm()
                elif choice == "3":
                    self.subscribe(self.DATA_TOPISCS["point_cloud"])
                    self.publish(self.COMMAND, "point_cloud")
                    self.looping()
                    
                elif choice == "4":
                    self.subscribe(self.DATA_TOPISCS["image"])
                    self.publish(self.COMMAND, "image")
                    
            #     elif choice.isdigit() and int(choice) in [option["value"] for option in menu_options]:
            #         chosen_option = [option for option in menu_options if option["value"] == int(choice)][0]
            #         print(f"You chose {chosen_option['name']}.")
                    
                else:
                    if self.status == 1:
                        print("Invalid choice. Please try again.")
        
        self.client.loop_stop()
    
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