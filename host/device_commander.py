import paho.mqtt.client as mqtt
import base64
import json
import rospy
import os
import config as CONFIG
import requests
import iot_status_checker as status_checker




# Define the command to be executed on the IoT device
# command = "/execute"

# # Define the headers for the HTTP request
# headers = {
#     "Content-Type": "application/json"
# }

# # Define the data to be sent in the HTTP request
# data = {
#     "param1": "value1",
#     "param2": "value2"
# }

# # Execute the command on the IoT device from the local machine
# local_url = f"http://{CONFIG.DEVICE.IP_ADDRESS}:{CONFIG.DEVICE.HTTP_PORT}{command}"
# response = requests.post(local_url, headers=headers, json=data)
# print(response.content)

# # Execute the command on the IoT device from the cloud
# cloud_url = "https://example.com/api/execute"
# response = requests.post(cloud_url, headers=headers, json=data)
# print(response.content)

# # Forward the command to the IoT device
# device_url = f"http://{iot_device_ip}:{iot_device_port}/command"
# response = requests.post(device_url, headers=headers, json={
#     "command": command,
#     "data": data
# })
# print(response.content)

# vibot = # 怎么建立连接?

# Set up the MQTT client
client = mqtt.Client()

# Define the function to check if the device is on
def check_device_power_status():
    
    # TODO: Demo code here, to modify later. 
    # 1. subscribe to the device's status topic, listening for 20s for example.  
    # 2. Once the localhost subscribed to the status topic, it sends commands to verify connectivity. This
    
    if status_checker.get_device_status():
        print("::: Device is ON")
    else:
        print("::: Device is OFF")
    
    # # Publish the check status command to the device topic
    # send_command("check_device_status")

# Define the function to check if the capturing service is on, and open it if necessary
def check_capturing_service():
    # Publish the check capturing service command to the device topic
    send_command("check_capturing_service")
    
# Define the function to open the capturing service
def open_capturing_service():
    # Publish the open capturing service command to the device topic
    send_command("open_capturing_service")

# Define the function to initiate data transfer
def start_data_transfer():
    # Publish the start command to the device topic
    send_command("start_transfer")


# Define the function to handle incoming messages
def on_message(client, userdata, message):
    # Decode the base64-encoded message payload
    data = base64.b64decode(message.payload)

    # Handle the data based on its type
    if message.topic.endswith("/image"):
        # If the message is an image, save it to disk
        with open("image.jpg", "wb") as f:
            f.write(data)
    elif message.topic.endswith("/pointcloud"):
        # If the message is a point cloud, process it
        process_pointcloud(data)
    elif message.topic.endswith("/data"):
        # If the message is a data packet, parse the JSON and process the data
        data_dict = json.loads(data)
        process_data(data_dict)


# Define the function to send commands to the device
def send_command(command):
    # Publish the command to the device topic
    client.publish( "/command", command)


# Define the function to capture data
def capture_data():
    # Publish the capture command to the device topic
    send_command("capture_data")

# Define the function to process point cloud data
def process_pointcloud(data):
    # Process the point cloud data
    pass

# Define the function to process data packets
def process_data(data_dict):
    # Extract the data type and payload from the data packet
    data_type = data_dict.get("type")
    data_payload = data_dict.get("payload")

    # Handle the data based on its type
    if data_type == "temperature":
        # If the data type is temperature, print the temperature value
        temperature = data_payload.get("value")
        print("Temperature: {:.2f} C".format(temperature))
    elif data_type == "humidity":
        # If the data type is humidity, print the humidity value
        humidity = data_payload.get("value")
        print("Humidity: {:.2f} %".format(humidity))
    else:
        # If the data type is unknown, print an error message
        print("Unknown data type: {}".format(data_type))

# TODO: 很重要的处理exception
# Define the on_disconnect() function to handle disconnection events
# Define the on_disconnect() function to handle disconnection events
def on_disconnect(client, userdata, rc):
    if rc != 0:
        print("Unexpected disconnection from MQTT broker. Retrying...")
        try:
            client.reconnect()
        except Exception as e:
            print("Error reconnecting to MQTT broker: {}".format(e))

# Define the function to adjust the device's settings
def adjust_settings(setting_name, setting_value):
    # Create a dictionary with the setting name and value
    setting_dict = {"name": setting_name, "value": setting_value}

    # Convert the dictionary to a JSON string
    setting_json = json.dumps(setting_dict)

    # Publish the setting change command to the device topic
    # try:
    #     client.publish(topic_prefix + "/settings", setting_json)
    # except Exception as e:
    #     print("Error adjusting settings: {}".format(e))

# define clear
def clear():
    if os.name == 'nt': 
        # If the operating system is Windows, then the function executes the cls command 
        # using the os.system() function to clear the terminal screen.
        os.system('cls')
        
    else:
    # If the operating system is not Windows (i.e., Mac or Linux), 
    # then the function executes the clear command instead.
        os.system('clear')


def menu():

    clear()
    print(
        '____________________________\n'
        ':::', CONFIG.APP.APPLICATION, CONFIG.APP.VERSION, ':::\n'
        ':::   ', CONFIG.APP.AUTHOR, '  :::\n'
        '¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯'
    )

    print("::: INFO :::")
    # print("::: POWER: " + denon.power)
    # print("::: NAME: " + denon.name)
    
    
    # Define the topic to subscribe/publish to
    topic_prefix = "vibot/commander"
    
    # TODO: do the security part later
    # # Set the username and password for the MQTT connection
    # client.username_pw_set(username="myuser", password="mypassword")

    # # Enable TLS encryption
    # client.tls_set(ca_certs="ca.crt", certfile="client.crt", keyfile="client.key", tls_version=ssl.PROTOCOL_TLSv1_2)
    # client.tls_insecure_set(False)

    # # Connect to the MQTT broker
    # client.connect("192.168.1.100", 8883)
    
    
    # Set up the MQTT client callbacks
    client.on_message = on_message

    # TODO: Integrate with the class
    # Connect to the MQTT broker
    client.connect("localhost", 1883)

    # Subscribe to the device topics
    client.subscribe(topic_prefix + "/#")

    # Start the MQTT client loop
    client.loop_start()

    print("\n::: MENU :::")
    print("::: 1 |=> Volume Control")
    print("::: 2 |=> Track Control")
    print("::: 3 |=> Input Control")
    print("::: 8 |=> Load Default Config")
    print("::: 9 |=> Quit\n")
    
    # Wait for user input
    command = input("::: Select point from menu: ")

    # Send the appropriate command to the device
    # if command in ['1']:
    #     clear()
    #     start_data_transfer()
    # elif command in ['2']:
    #     capture_data()
    # elif command in ['3']:
    #     check_device_status()
    # elif command in ['4']:
    #     check_capturing_service()
    # elif command in ['5']:
    #     others()
    # elif command in ['9']:
    #     exit(code="Exiting...")
    # else:
    #     print("Please select point from menu.")
    #     time.sleep(2)
    #     clear()
    #     menu()
       
        

    # Stop the MQTT client loop
    client.loop_stop()
    

    
if __name__ == '__main__':
    try:
        check_device_power_status()
    except rospy.ROSInterruptException:
        """
        If ROS interrupt exception is raised (e.g., if the user
        presses Ctrl-C to stop the program), the exception is
        caught and ignored. 
        """
        pass