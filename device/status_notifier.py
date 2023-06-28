import paho.mqtt.client as mqtt
import time

# Define MQTT client callback functions
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    # Publish initial status update
    client.publish("iot_device/status", "connected")

def on_message(client, userdata, msg):
    print("Received message: "+str(msg.payload.decode()))
    # If command is "status_check", respond with current status
    if msg.payload.decode() == "status_check":
        client.publish("iot_device/command_response", "status_ok")

# Create MQTT client instance and connect to broker
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect("localhost", 1883, 60)

# Start MQTT client loop
client.loop_start()

# Send initial status update
client.publish("iot_device/status", "connected")

# Monitor for incoming messages and respond to commands indefinitely
while True:
    time.sleep(1)

# Stop MQTT client loop and disconnect from broker
client.loop_stop()
client.disconnect()