import paho.mqtt.client as mqtt
import threading
import atexit
import time


class StatusNotifierBridge:
    def __init__(self, command_topic = "iot_device/command", 
                 client_id="device", user_id="", password="", 
                 host="43.133.159.102", port=1883, keepalive=60, qos=0):
       
        self.command_topic = command_topic
        self.client_id = client_id
        self.user_id = user_id
        self.password = password
        self.host = host
        self.port = port
        self.keepalive = keepalive
        self.qos = qos
        

        self.disconnect_flag = False
        self.rc = 1
        self.timeout = 0

        # Create the MQTT client object, set the username and password, and register the callback functions
        self.client = mqtt.Client(self.client_id, clean_session=True)
        self.client.username_pw_set(self.user_id, self.password)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect

        # Connect to the broker
        self.connect()

    def connect(self):
        """
        Connect to the MQTT broker
        """
        while self.rc != 0:
            try:
                self.rc = self.client.connect(self.host, self.port, self.keepalive)
            except:
                # If the connection fails, wait for 2 seconds before trying again
                print("Connection failed")
                
            time.sleep(2)
            self.timeout += 2

    def disconnect(self):
        """
        Disconnect from the MQTT broker
        """
        print("Disconnecting")
        self.disconnect_flag = True
        self.client.disconnect()
        
    def msg_process(self, msg):
        """
        Process incoming MQTT messages
        """
        print("processing message")
        msg = str(msg.payload.decode())
        if msg == "status_check":
            self.publish("iot_device/command_response", "status_ok")
            print("sent to iot_device/command_response")
        pass

    def looping(self, loop_timeout=30):
        """
        Start the MQTT client's event loop
        :param loop_timeout: The maximum time to wait for incoming messages before returning
        """
        # self.client.loop(loop_timeout)
        self.client.loop_forever()

    def on_connect(self, client, userdata, flags, rc):
        """
        Callback function called when the client successfully connects to the broker
        """
        print(f"Connected to MQTT broker with result code {str(rc)}")
        self.client.subscribe(self.command_topic)
        self.timeout = 0
        
        # start a thread to publish status
        status_thread = threading.Thread(target=self.publish_status)
        status_thread.start()
        atexit.register(self.stop_publish_status_thread, status_thread)
    
    def publish_status(self):
        """
        Continuously publish device status every second
        """
        for i in range(20):
            self.publish("iot_device/status", "connected")
            print("daemon process publishes to iot_device/status indicating it's there")
            time.sleep(3)
            
    def stop_publish_status_thread(self, thread):
        print("Stopping thread")
        thread.join()
        
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

    def on_message(self, client, userdata, msg):
        """
        Callback function called when an incoming message is received
        """
        print("iot device obtains the message in iot_device/command")
        self.msg_process(msg)

    def unsubscribe(self):
        """
        Unsubscribe from the MQTT topic
        """
        print("Unsubscribing")
        self.client.unsubscribe(self.command_topic)

    def disconnect(self):
        """
        Disconnect from the MQTT broker
        """
        print("Disconnecting")
        self.disconnect_flag = True
        self.client.disconnect()

    
    def publish(self, topic, message):
        """
        Publish a message to the MQTT broker
        :param message: The message to publish
        """
        self.client.publish(topic, message)

    def hook(self):
        """
        Gracefully shut down the MQTT client
        """
        self.unsubscribe()
        self.disconnect()
        print("Shutting down")

    def get_timeout(self):
        """
        Get the amount of time elapsed since the connection attempt started
        """
        return self.timeout

if __name__ == "__main__":
    
    # Set up MQTT client and callbacks
    sn = StatusNotifierBridge()
    sn.looping()
    
    # Continuously publish device status every 5 seconds

    # Stop MQTT client loop and disconnect from broker
    

