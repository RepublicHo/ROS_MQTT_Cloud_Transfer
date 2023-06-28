import paho.mqtt.client as mqtt
import time
import config as CONFIG

# Define MQTT client callback functions
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    # Subscribe to the device's status topic
    client.subscribe("iot_device/status")

def on_message(client, userdata, msg):
    print("Received message: " + str(msg.payload.decode()))
    # Verify connectivity by sending a command to the device
    client.publish("iot_device/command", "status_check")

def on_command_response(client, userdata, msg):
    print("Received command response: " + str(msg.payload.decode()))
    # Set flag indicating that the device is responding to commands
    userdata['command_response_received'] = True

# Define function to check device status and connectivity
def check_device_status(client, userdata, timeout=30):
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
    if userdata['status_received'] and userdata['command_response_received']:
        return True
    else:
        return False

# Define function to get device status and return verification status
def get_device_status(timeout=30):
    # Create MQTT client instance with userdata dictionary initialized
    userdata = {'status_received': False, 'command_response_received': False}
    client = mqtt.Client(userdata=userdata)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(CONFIG.CONNECTION.BROKER, CONFIG.CONNECTION.PORT, 60)

    # Subscribe to the device's status topic and set on_status_received callback function
    client.subscribe("iot_device/status")
    client.message_callback_add("iot_device/status", lambda client, userdata, msg: userdata.update({'status_received': True}))

    # Wait for timeout seconds or until a status update is received
    start_time = time.time()
    while (time.time() - start_time) < timeout:
        if userdata['status_received']:
            break
        time.sleep(1)

    # If a status update was received, check device status and return verification status
    if userdata['status_received']:
        # Subscribe to the device's command response topic and set on_command_response callback function
        client.subscribe("iot_device/command_response")
        client.message_callback_add("iot_device/command_response", on_command_response)

        # Send command to device to verify connectivity
        client.publish("iot_device/command", "status_check")

        # Wait for timeout seconds for command response
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            if userdata['command_response_received']:
                break
            time.sleep(1)

        # Call check_device_status function to return device verification status
        verification_status = check_device_status(client, userdata, timeout=timeout)

        # Disconnect from broker
        client.disconnect()

        # Return device verification status
        return verification_status
    # If no status update was received, disconnect from broker and return False
    else:
        client.disconnect()
        return False