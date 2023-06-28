import paho.mqtt.client as mqtt
import time
import config as CONFIG
import paho.mqtt.client as mqtt
import time
import paho.mqtt.client as mqtt
import time

# Define MQTT client callback functions
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    # Subscribe to the device's status topic
    client.subscribe("iot_device/status")

def on_message(client, userdata, msg):
    print("Received message: "+str(msg.payload.decode()))
    # Verify connectivity by sending a command to the device
    client.publish("iot_device/command", "status_check")

def on_command_response(client, userdata, msg):
    print("Received command response: "+str(msg.payload.decode()))
    # Set flag indicating that the device is responding to commands
    global command_response_received
    command_response_received = True

# Define function to check device status and connectivity
def check_device_status(timeout=30):
    # Create MQTT client instance and connect to broker
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(CONFIG.CONNECTION.BROKER, CONFIG.CONNECTION.PORT, 60)

    # Start MQTT client loop
    client.loop_start()

    # Monitor status updates for timeout seconds
    start_time = time.time()
    while (time.time() - start_time) < timeout:
        pass

    # Stop MQTT client loop and disconnect from broker
    client.loop_stop()
    client.disconnect()

    # Return boolean value indicating device verification status
    if status_received and command_response_received:
        return True
    else:
        return False

# Initialize status_received and command_response_received variables
status_received = False
command_response_received = False

# Define MQTT client callback function to set status_received variable
def on_status_received(client, userdata, msg):
    global status_received
    status_received = True

# Define function to check device status and return verification status
def get_device_status(timeout=30):
    # Create MQTT client instance and connect to broker
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(CONFIG.CONNECTION.BROKER, CONFIG.CONNECTION.PORT, 60)

    # Start MQTT client loop
    client.loop_start()

    # Subscribe to the device's status topic and set on_status_received callback function
    client.subscribe("iot_device/status")
    client.message_callback_add("iot_device/status", on_status_received)


    # Wait for timeout seconds or until a status update is received
    start_time = time.time()
    while (time.time() - start_time) < timeout:
        if status_received:
            break
        time.sleep(1)

    # Stop MQTT client loop and disconnect from broker
    client.loop_stop()
    client.disconnect()

    # If a status update was received, check device status and return verification status
    if status_received:
        # Create MQTT client instance and connect to broker
        client = mqtt.Client()
        client.on_connect = on_connect
        client.on_message = on_message
        client.connect(CONFIG.CONNECTION.BROKER, CONFIG.CONNECTION.PORT, 60)

        # Start MQTT client loop
        client.loop_start()

        # Subscribe to the device's command response topic and set on_command_response callback function
        client.subscribe("iot_device/command_response")
        client.message_callback_add("iot_device/command_response", on_command_response)

        # Send command to device to verify connectivity
        client.publish("iot_device/command", "status_check")

        # Wait for timeout seconds for command response
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            if command_response_received:
                break
            time.sleep(1)

        # Stop MQTT client loop and disconnect from broker
        client.loop_stop()
        client.disconnect()

        # Call check_device_status function to return device verification status
        verification_status = check_device_status(timeout=timeout)

        # Return device verification status
        return verification_status
    # If no status update was received, return False
    else:
        return False