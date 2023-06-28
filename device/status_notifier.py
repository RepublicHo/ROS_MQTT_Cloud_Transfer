import paho.mqtt.client as mqtt
import config as CONFIG
import time

# please make it like a daemon process
# Define MQTT client callback functions
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    # Publish initial status update
    client.subscribe("iot_device/command")
    

def on_message(client, userdata, msg):
    print("Received message: "+str(msg.payload.decode()))
    # If command is "status_check", respond with current status
    if msg.payload.decode() == "status_check":
        for i in range(10):
            client.publish("iot_device/command_response", "status_ok")
            print("sent to iot_device/command_response")
            time.sleep(1)

if __name__ == "__main__":
    # Create MQTT client instance and connect to broker
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(CONFIG.CONNECTION.BROKER, CONFIG.CONNECTION.PORT, 60)


    # Start MQTT client loop to listen for incoming messages
    client.loop_start()

    # Continuously publish status update every 5 seconds
    while True:
        client.publish("iot_device/status", "connected")
        print("sent to iot_device/status")
        time.sleep(5)

    # Stop MQTT client loop and disconnect from broker
    client.loop_stop()
    client.disconnect()

