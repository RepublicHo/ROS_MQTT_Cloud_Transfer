import paho.mqtt.client as mqtt
import time

# please make it like a daemon process
# Define MQTT client callback functions
def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker with result code "+str(rc))
    # Publish initial status update
    client.subscribe("iot_device/command")
    client.loop_start()
    for i in range(5):
        # Publish a message to the "example/topic" topic every 5 seconds
        client.publish("iot_device/status", "connected")
        print("daemon process publishes to iot_device/status indicating it's there")
        time.sleep(1)
    

def on_message(client, userdata, msg):
    # print("Received message: "+str(msg.payload.decode()))
    # If command is "status_check", respond with current status
    # if msg.payload.decode() == "status_check":
    print("iot device obtains the message in iot_device/command")
    # Subscribe to command topic
    for i in range(5):
        send_status_updates(client)
        time.sleep(1)

def connect(self):
    """
    Connect to the MQTT broker
    """
    while self.rc != 0:
        try:
            self.rc = self.client.connect()
        except:
            # If the connection fails, wait for 2 seconds before trying again
            print("Connection failed")
            print("Kindly note that please have mosquitto properly installed and configured. \n From Mosquitto 2 and higher its only possible to login with localhost on that device. "
                    +"\nFor another device you have to change some settings in mosquitto.conf. ")
        time.sleep(2)
        self.timeout += 2

def on_disconnect(self, client, userdata, rc):
    """
    Callback function called when the client is unexpectedly disconnected from the broker
    """
    if rc != 0:
        if not self.disconnect_flag:
            print("Unexpected disconnection.")
            print("Trying reconnection")
            self.rc = rc
            self.connect() 

def send_status_updates(client):
    # Send a series of status updates every second for 10 seconds
    
    client.publish("iot_device/command_response", "status_ok")
    print("sent to iot_device/command_response")
    time.sleep(1)
        
if __name__ == "__main__":
    
    # Set up MQTT client and callbacks
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    # Connect to MQTT broker and start client loop
    client.connect()
    
    # Continuously publish device status every 5 seconds

    # Stop MQTT client loop and disconnect from broker
    client.loop_forever()

