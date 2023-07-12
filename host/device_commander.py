import paho.mqtt.client as mqtt
import numpy as np
import time
import sys
import queue
import datetime
from message_processor import ImageProcessor, PointCloudProcessor
from sensor_msgs.msg import Image
import rospy
import threading
import os
import logging
import json
import config as CONFIG
import iot_status_checker as isc

from bridge import Bridge

# Yes, there are more advanced systems that can be used to stop the data transfer in a more graceful and efficient way. Here are some examples:

# 应该用这个 Use a dedicated command topic: Instead of waiting for user input on the terminal, you can create a dedicated MQTT topic for sending control commands to the IoT device. The device can listen to this topic and stop the data transfer when it receives a stop command. This approach allows the user to stop the data transfer from any device that has access to the MQTT broker, not just the device running the program.


class DeviceCommander(Bridge):
    # Define class constant 
    DEFAULT_MQTT_TOPIC = "/iot_device/command_response"
    
    def __init__(
            self, 
            mqtt_topic=DEFAULT_MQTT_TOPIC,
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

        self.heartbeat_thread = None
        self.heartbeat_running = False
        self._lock = threading.Lock()
        self.last_heartbeat_time = time.time()
        self.HEARTBEAT_TIMEOUT = 60 # seconds
        
        # connected is used for checking if the host can connected to the MQTT broker
        # TODO: if lose connection, set it to false and reconnect once again. 
        self.connected = False
        
        # constants for brokers
        self.DEVICE_HEARTBEAT = "/iot_device/heartbeat"
        self.COMMAND = "/iot_device/command"
        self.COMMAND_RESPONSE = "/iot_device/command_response"
        
        # Instantiate two message processor classes for image and point cloud.
        self.image_processor = ImageProcessor(self.DATA_TOPISCS["image"], host=CONFIG.CONNECTION.BROKER, port=1883)
        self.pc_processor = PointCloudProcessor(self.DATA_TOPISCS["point_cloud"], host=CONFIG.CONNECTION.BROKER, port=1883)
        
        
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
        
        self.vio_enabled = False
        
        # TODO: Not yet used. 
        self.user_input_queue = queue.Queue()
        self.status = 0 # 0 for not connected, 1 for connected, 2 for timeout or other bugs
        
        
        super().__init__(mqtt_topic, client_id, user_id, 
                         password, host, port, keepalive, qos)
     
    def start_check_heartbeat(self):
        with self._lock:
            if not self.heartbeat_running:
                self.heartbeat_running = True
                self.heartbeat_thread.start()
    
    def restart_check_heartbeat(self):
        with self._lock:
            self.heartbeat_running = True
        
    def stop_check_heartbeat(self):
        with self._lock:
            self.heartbeat_running = False
            
    def end_check_heartbeat(self):
        self.stop_check_heartbeat()
        self.heartbeat_thread.join() 
        
    def check_heartbeat(self):
        """Checks if the device has received a heartbeat message from the cloud within the specified timeout.

        If the last heartbeat time is older than the heartbeat timeout duration, sets the status to 0 (timeout).

        To stop this function, set the status to a value other than 1.
        """
        while self.status == 1:
            with self._lock:
                if self.heartbeat_running:     
                    # test code
                    print(f"time now: {time.time()}, and last_heartbeat_time: {self.last_heartbeat_time}")
                    time_since_last_heartbeat = time.time() - self.last_heartbeat_time
                    if time_since_last_heartbeat > self.HEARTBEAT_TIMEOUT:
                        self.status = 0
                        self.logger.warning("Heartbeat timed out")
                        
            time.sleep(1)
            
        self.logger.info("Heartbeat check stopped")
        self.hook()    
 
    # Define the function to handle incoming messages
    def on_message(self, client, userdata, msg):   
        
        if msg.topic == self.DEVICE_HEARTBEAT:
            self.last_heartbeat_time = time.time()
            # test code: print message when it hearts the heartbeat. 
            self.logger.info("Heartbeat received, updated last_heartbeat: {self.last_heartbeat_time}")
               
        elif msg.topic == self.DATA_TOPISCS["point_cloud"]:
            # point cloud message will be handled by point cloud processor instance 
            pass
        
        elif msg.topic == self.DATA_TOPISCS["image"]:
            # image message will be handled by image processor instance 
            pass
            
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
            self.logger.info(f"Received JSON message: type {json_msg['type']} and code {json_msg['code']}")
            
            if json_msg['type'] == "enable_vio" and json_msg['code'] == 200:
                self.vio_enabled = True
                
            elif json_msg['type'] == "disable_vio" and json_msg['code'] == 200:
                self.vio_enabled = False
                
            elif json_msg['type'] == "start_point_cloud_transfer" and json_msg['code'] == 200:
                self.logger.debug("Device responses that it will be starting point cloud transfer")
                # self.pc_processor.start_processing()
                
            elif json_msg['type'] == "start_image_transfer" and json_msg['code'] == 200:
                self.logger.debug("Device responses that it will be starting image transfer")
                # self.image_processor.start_processing()

            elif json_msg['type'] == "end_point_cloud_transfer" and json_msg['code'] == 200:
                self.logger.debug("Device responses that it will be ending point cloud transfer")
                # self.pc_processor.stop_processing()
                
            elif json_msg['type'] == "end_image_transfer" and json_msg['code'] == 200:
                self.logger.debug("Device responses that it will be ending image transfer")
                # self.image_processor.stop_processing()
            
            else:
                self.logger.warning(f"A JSON message {json_msg['type']} with code {json_msg['code']} is received unexpectedly!")
        
        else:
            self.logger.warning("An invalid JSON message is received! Please check!")
    
    def check_device_power_status(self):
        
        status_checker = isc.StatusChecker(client_id = "status_checker", host="43.133.159.102", port=1883)
        
        status = status_checker.get_device_status()
        print("zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz")
        print("Status code: ", status)
        print("zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz")
        if status:
            print("::: Device is ON")
            return 1
        else:
            print("::: Device is OFF, please check the device")
            return 0
    
    

    
    # Define the function to enable the vio capturing algorithm
    def enable_vio_algorithm(self):
        
        self.publish(self.COMMAND, "enable_vio_service")
        timeout = time.time() + 20  # Wait for up to 20 seconds

        while time.time() < timeout:
            if self.vio_enabled:
                self.logger.debug("Successfully enabled vio service")
                return "vio algorithm is enabled"
            time.sleep(1)

        self.logger.debug("Failed to enable vio service")
        return "vio algorithm has trouble being enabled"          
        
        
    # Define the function to disable the vio capturing algorithm
    def disable_vio_algorithm(self):
        
        # Wait for 10 seconds and see if we can subscribe to message from topic RESPONSE
        self.publish(self.COMMAND, "disable_vio_service")
        timeout = time.time() + 20  # Wait for up to 20 seconds

        while time.time() < timeout:
            if not self.vio_enabled:
                self.logger.debug("Successfully disabled vio service")
                return "vio algorithm is disabled"
            time.sleep(1)

        self.logger.debug("Failed to disable vio service")
        return "vio algorithm has trouble being disabled"  
    

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
        with self._lock:
            self.status = self.check_device_power_status()
        print("=====================Status==================")
        self.logger.debug(f"The status code is {self.status}")
        print()
        
        if self.status == 1:
            
            # Continuously subscribe to heartbeat topic in the cloud.  
            self.subscribe(self.DEVICE_HEARTBEAT)
            
            
            self.client.loop_start()
            
            self.last_heartbeat_time = time.time()
            self.heartbeat_thread = threading.Thread(target=self.check_heartbeat, daemon=True)
            self.start_check_heartbeat()
            
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
                {"name": "Start Point Cloud Transfer", "value": 3},
                {"name": "End Point Cloud Transfer", "value": 4},
                {"name": "Start Image Transfer", "value": 5},
                {"name": "End Image Transfer", "value": 6},
                {"name": "Exit", "value": 0}
            ]
                
            # Display the menu and prompt the user for input
            while self.status == 1:
                
                last_command_result = ""
                
            # Display the menu options
                print("\n::: Menu :::")
                for option in menu_options:
                    print(f"{option['value']}. {option['name']}")
                
                choice = input("Enter your choice: ")
                # Handle the user's choice
                if choice == "0":
                    self.status = 0
                    print("Exiting...")
                    break
                
                elif choice == "1":
                    last_command_result = self.enable_vio_algorithm()
                    
                elif choice == "2":
                    last_command_result = self.disable_vio_algorithm()
                    
                elif choice == "3":
                    # self.subscribe(self.DATA_TOPISCS["point_cloud"])
                    self.publish(self.COMMAND, "start_point_cloud_transfer")
                    last_command_result = "Point cloud transfer starts. Heartbeat checking is paused temporarily. "
                    self.stop_check_heartbeat()
                    
                elif choice == "4":
                    # self.subscribe(self.DATA_TOPISCS["image"])
                    self.publish(self.COMMAND, "end_point_cloud_transfer")
                    last_command_result = "Point cloud transfer ends. Heartbeat checking is resumed. "
                    self.restart_check_heartbeat()
                    
                elif choice == "5":
                    # self.unsubscribe(self.DATA_TOPISCS["point_cloud"])
                    self.publish(self.COMMAND, "start_image_transfer")
                    last_command_result = "Image transfer starts. Heartbeat checking is paused temporarily. "
                    self.stop_check_heartbeat()
                    
                elif choice == "6":
                    # self.unsubscribe(self.DATA_TOPISCS["image"])
                    self.publish(self.COMMAND, "end_image_transfer")
                    last_command_result = "Image transfer ends. Heartbeat checking is resumed. "
                    self.restart_check_heartbeat()
                    
                else:
                    last_command_result = "Invalid choice. Please try again."
             
                if self.status == 0:
                    last_command_result = "Detected the device failed to connect to MQTT broker. Please check the device. "
                    self.stop_check_heartbeat()
                    
                print(last_command_result)
                
        
        self.client.loop_stop()
    
    # Define the function to initiate data transfer
    def start_data_transfer(self, data):
        self.publish()

def main():
    # rospy.init_node('device_commander', anonymous=True)
    
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
