import paho.mqtt.client as mqtt
import time
import config as CONFIG

# Define MQTT client callback functions
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    # Subscribe to the device's status topic    
    client.subscribe("iot_device/status")
    client.subscribe("iot_device/command_response")

def on_message(client, userdata, msg):
    print("Received message: " + str(msg.payload.decode()))
    # Verify connectivity by sending a command to the device
    client.publish("iot_device/command", "status_check")

def on_command_response(client, msg):
    
    print("Received command response: " + str(msg.payload.decode()))
    # Set flag indicating that the device is responding to commands
    dict['command_response_received'] = True

def on_status_received(client, userdata, msg):
    
    print("Received status update: " + str(msg.payload.decode()))
    # Set flag indicating that a status update has been received
    dict['status_received'] = True

# Define function to check device status and connectivity
def check_device_status(client, timeout=20.0):
    global dict
    # Start MQTT client loop
    client.loop_start()

    # Stop MQTT client loop and disconnect from broker
    client.loop_stop()
    client.disconnect()

    # Return boolean value indicating device verification status
    if dict['status_received'] and dict['command_response_received']:
        print("host received data on iot_device/command_response")
        return True
    else:
        return False

# Define function to get device status and return verification status
def get_device_status(timeout=20):
    # Create MQTT client instance with global dict dictionary
    global dict
    dict = {'status_received': False, 'command_response_received': False}
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(CONFIG.CONNECTION.BROKER, CONFIG.CONNECTION.PORT, 60)
    
    client.message_callback_add("iot_device/status", on_status_received)
    
    print("hello")
    time_wait = time.time() + 5   # Loop for 5 seconds
    while time.time() < time_wait:
        client.loop()
    
    # Wait for timeout seconds or until a status update is received
    start_time = time.time()
    print(4)
    while (time.time() - start_time) < timeout:
        if dict['status_received']:
            break
        time.sleep(1)
    print(5)
    # If a status update was received, check device status and return verification status
    if dict['status_received']:
        print("host received data on iot_device/status")
        # Subscribe to the device's command response topic and set on_command_response callback function
        client.subscribe("iot_device/command_response")
        
        print(8)
        # Send command to device to verify connectivity
        client.publish("iot_device/command", "status_check")
        print(9)
        print(7)
        client.message_callback_add("iot_device/command_response", on_command_response)
        start_time = time.time()
        # Wait for timeout seconds for command response
        client.message_callback_add("iot_device/status", on_status_received)
        
        while (time.time() - start_time) < timeout:
            if dict['command_response_received']:
                break
            time.sleep(1)
        print(10)
        # Call check_device_status function to return device verification status
        verification_status = check_device_status(client, dict)

        # Disconnect from broker
        client.disconnect()

        # Return device verification status
        return verification_status
    # If no status update was received, disconnect from broker and return False
    else:
        client.disconnect()
        return False

# test code  
if __name__ == '__main__':
    print(get_device_status())