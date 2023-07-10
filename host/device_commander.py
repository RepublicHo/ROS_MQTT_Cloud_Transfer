import paho.mqtt.client as mqtt
import numpy as np
import time
import sys
import queue
import datetime
from sensor_msgs.msg import Image
import rospy
import threading
import os
import struct
import logging
import json
import binascii
import config as CONFIG
import iot_status_checker as isc


from bridge import Bridge

# Yes, there are more advanced systems that can be used to stop the data transfer in a more graceful and efficient way. Here are some examples:

# 应该用这个 Use a dedicated command topic: Instead of waiting for user input on the terminal, you can create a dedicated MQTT topic for sending control commands to the IoT device. The device can listen to this topic and stop the data transfer when it receives a stop command. This approach allows the user to stop the data transfer from any device that has access to the MQTT broker, not just the device running the program.

# Use a watchdog timer: A watchdog timer is a hardware or software timer that is used to detect and recover from errors in a system. In this case, you can use a software watchdog timer to monitor the data transfer and reset the timer periodically. If the timer expires without being reset, it indicates that the data transfer has stopped, and the device can stop the transfer gracefully.

# Use a state machine: A state machine is a mathematical model that describes the behavior of a system. In this case, you can use a state machine to model the data transfer process and the control commands. The device can transition between different states based on the control commands and the status of the data transfer. This approach allows for more complex control logic and error handling.

class DeviceCommander(Bridge):
    
    def __init__(
            self, 
            client_id = "commander", 
            user_id="", 
            password="", 
            host="localhost", 
            port=1883, 
            keepalive=60, 
            qos=0
    ):

        # topics in the mqtt broker 
        self.DATA_TOPISCS = {"point_cloud": "/data/point_cloud",
                             "image": "/data/img"}

        # connected is used for checking if the host can connected to the MQTT broker
        # TODO: if lose connection, set it to false and reconnect once again. 
        self.connected = False
        
        # constants for brokers
        self.DEVICE_HEARTBEAT = "/iot_device/heartbeat"
        self.COMMAND = "/iot_device/command"
        self.COMMAND_RESPONSE = "/iot_device/command_response"
        
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
        file_handler = logging.FileHandler("device_commander.log")
        file_handler.setLevel(logging.INFO)
        file_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)
        
        # TODO: Not yet used. 
        self.lock = threading.Lock()
        self.user_input_queue = queue.Queue()
        self.status = 0 # 0 for not connected, 1 for connected, 2 for timeout or other bugs
        self.last_heartbeat_time = time.time()
        self.HEARTBEAT_TIMEOUT = 30 # seconds
        self.vio_enabled = False
        

        super().__init__(self.COMMAND_RESPONSE, client_id, user_id, 
                         password, host, port, keepalive, qos)
    
    def on_connect(self, client, userdata, flags, rc):
        """
        Callback function called when the client successfully connects to the broker
        """
        logging.info(f"Connected to MQTT broker with result code {str(rc)}")
        
        # Subscribed to the command respomse topic
        self.subscribe(self.COMMAND_RESPONSE)
        
        # Set the connected (to MQTT broker) flag to true.
        self.connected = True
        self.timeout = 0
        
 
    # Define the function to handle incoming messages
    def on_message(self, client, userdata, msg):
        
        # testing code： Decode the base64-encoded message payload
        # data = base64.b64decode(msg.payload)
        # print("msg: " + msg)
        # print("data: " + data)
        
        # testing code
        if msg.topic != self.DEVICE_HEARTBEAT:
            self.logger.info("Commander received message on topic " + msg.topic + " with payload size " + str(len(msg.payload)))
        
        
        if msg.topic == self.DEVICE_HEARTBEAT:
            self.last_heartbeat_time = time.time()
            
            # test code: print message when it hearts the heartbeat. 
            print("Heartbeat received, updated last_heartbeat", self.last_heartbeat_time)    
        elif msg.topic == self.DATA_TOPISCS["point_cloud"]:
            self.process_point_cloud(msg)
        
        elif msg.topic == self.DATA_TOPISCS["image"]:
            self.process_image(msg)
            
        elif msg.topic == self.COMMAND_RESPONSE:  
            self.msg_process(msg)
        
        else: # If message is from an unknown topic, it serves as a reminder. 
            self.logger.warning(f"We unexpectedly received a message from {msg.topic} topic")
            self.logger.warning("This could be a threat! ")
    
    def is_json(self, string):
        try:
            json_object = json.loads(string)
        except ValueError:
            return False
        return True
    
    def msg_process(self, msg):
        '''
        TODO: You can add more functionality here if needed. 
        '''
        
        if self.is_json(msg.payload.decode()):
            # Decode the message payload from JSON format
            json_msg = json.loads(msg.payload.decode())
            self.logger.info("Received JSON message: ", json_msg)
            
            if json_msg['type'] == "enable_vio" and json_msg['code'] == 200:
                self.vio_enabled = True
            elif json_msg['type'] == "disable_vio" and json_msg['code'] == 200:
                self.vio_enabled = False
            elif json_msg['type'] == "point_cloud_transfer" and json_msg['code'] == 200:
                self.subscribe(topic=self.DATA_TOPISCS["point_cloud"])
            elif json_msg['type'] == "image_transfer" and json_msg['code'] == 200:
                self.subscribe(topic=self.DATA_TOPISCS["image"])
        
            # TODO: 完善停止传输
            elif json_msg['type'] == "stop_point_cloud_transfer" and json_msg['code'] == 200:
                self.unsubscribe(topic=self.DATA_TOPISCS["point_cloud"])
            elif json_msg['type'] == "stop_image_transfer" and json_msg['code'] == 200:
                self.unsubscribe(topic=self.DATA_TOPISCS["image"])
            
            else:
                self.logger.warning(f"A JSON message {json_msg['type']} with code {json_msg['code']} is received unexpectedly!")
        else:
            self.logger.warning("An invalid JSON message is received! Please check!")
      
        
    def process_point_cloud(self, msg):
        """
        Function to process the point cloud data received from the IoT device
        TODO: properly save/visualize the point cloud data as appropriate.
         
        """
        try:
            self.logger.debug("Received message on topic " + msg.topic + " with payload size " + str(len(msg.payload)))
            
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
            
    
    def process_image(self, msg):
        """
        Function to process the image data received from the IoT device
        TODO: properly save/visualize the image data. 
        """
        try:
            print("Received message on topic " + msg.topic + " with payload size " + str(len(msg.payload)))
            
        except Exception as e:
            self.logger.error("Error occurs when processing point cloud: {}".format(e))


    
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
        time.sleep(2)
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
        time.sleep(2)
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
                {"name": "Start transferring Point Cloud", "value": 3},
                {"name": "Start transferring Image", "value": 4},
                {"name": "End transferring Point Cloud", "value": 5},
                {"name": "End transferring Image", "value": 6},
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
                    self.status = 0
                    print("Exiting...")
                    break
                
                elif choice == "1":
                    self.enable_vio_algorithm()
                    
                elif choice == "2":
                    self.disable_vio_algorithm()
                    
                elif choice == "3":
                    self.subscribe(self.DATA_TOPISCS["point_cloud"])
                    self.publish(self.COMMAND, "start_point_cloud_transfer")
                    time.sleep(30)
                    
                elif choice == "4":
                    self.subscribe(self.DATA_TOPISCS["image"])
                    self.publish(self.COMMAND, "start_image_transfer")
                    
                    
                elif choice == "5":
                    self.unsubscribe(self.DATA_TOPISCS["point_cloud"])
                    self.publish(self.COMMAND, "end_point_cloud_transfer")
                    time.sleep(30)
                    
                elif choice == "6":
                    self.unsubscribe(self.DATA_TOPISCS["image"])
                    self.publish(self.COMMAND, "end_image_transfer")
                    
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