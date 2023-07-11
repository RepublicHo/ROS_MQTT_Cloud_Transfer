#!/usr/bin/python
import paho.mqtt.client as mqtt
import time
import logging

class Bridge:
    def __init__(self, mqtt_topic, client_id="bridge", user_id="", password="", 
                 host="localhost", port=1883, keepalive=60, qos=0):
        """
        Constructor method for the bridge class
        :param mqtt_topic: The topic to publish/subscribe to
        Most case we foucs on one topic to publish/subscribe to.
        If we want to focus on some specific topic, we have to specify the topic in the methods of publish and subscribe. 
        :param client_id: The ID of the client
        :param user_id: The user ID for the broker
        :param password: The password for the broker
        :param host: The hostname or IP address of the broker
        :param port: The port number of the broker
        :param keepalive: The keepalive interval for the client
        :param qos: The Quality of Service that determines the level of guarantee 
        for message delivery between MQTT client and broker. 
        """
        # Validate user inputs
        if "#" in mqtt_topic or "+" in mqtt_topic:
            raise ValueError("Publish topic cannot contain wildcards")
        if qos not in [0, 1, 2]:
            raise ValueError("QoS level must be 0, 1, or 2")
        if keepalive <= 0:
            raise ValueError("Keepalive interval must be a positive integer")
        if not isinstance(port, int):
            raise ValueError("Port must be an integer!")
        
        self.mqtt_topic = mqtt_topic
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
        self.client.on_unsubscribe = self.on_unsubscribe
        self.client.on_subscribe = self.on_subscribe

        # Connect to the broker
        self.connect()

    def connect(self):
        """
        Connect to the MQTT broker
        """
        while self.rc != 0:
            try:
                self.rc = self.client.connect(self.host, self.port, self.keepalive)
            except Exception as e:
                # If the connection fails, wait for 2 seconds before trying again
                # If the connection fails, log the error and wait for some time before trying again.
                logging.error(f"Failed to connect to MQTT broker: {e}")
                logging.info("Waiting for 2 seconds before retrying...")
                time.sleep(2)
                self.timeout += 2
                
                # Print some suggestions for potential soluations
                print("---\nSuggestions: ")
                print("1. Check the WIFI connection. MQTT return code(rc) currently is {self.rc} \n")
                print("2. Check the MQTT version in the cloud. You might encounter local loopback monitoring issue in mosquitto 2 and higher. (I encountered it in Aliyun). "
                      +"\n You may downgrade MQTT to 1.6 stable or configure mosquitto.conf as appropriate.")
                print("3. Wait for a while, or rerun the program. \n")

    def msg_process(self, msg):
        """
        Process incoming MQTT messages
        """
        # TODO: you can inherit this class and focus on coding in this method
        pass

    def looping(self, loop_timeout=60):
        """
        Start the MQTT client's event loop
        :param loop_timeout: The maximum time to wait for incoming messages before returning
        """
        self.client.loop(loop_timeout)

    def on_connect(self, client, userdata, flags, rc):
        """
        Callback function called when the client successfully connects to the broker
        """
        logging.info(f"Connected to MQTT broker with result code {str(rc)}")
        self.client.subscribe(self.mqtt_topic)
        self.timeout = 0
        
    def subscribe(self, topic=None):
        """
        Subscribe to a topic. 
        :param topic: The topic to subscribe to.Optional, defaults to the topic passed in the constructor method.
        """
        if topic is None:
            topic == self.mqtt_topic
        self.client.subscribe(topic)
        
    def on_disconnect(self, client, userdata, rc):
        """
        Callback function called when the client is unexpectedly disconnected from the broker
        """
        if rc != 0:
            if not self.disconnect_flag:
                logging.warning("Unexpected disconnection.")
                logging.warning("Trying reconnection")
                self.rc = rc
                self.connect()

    def on_message(self, client, userdata, msg):
        """
        Callback function called when an incoming message is received
        """
        self.msg_process(msg)


    def unsubscribe(self, topic=None):
        """
        Unsubscribe from the MQTT topic. Optional, defaults to the topic passed in the constructor method.
        """
        if topic is None:
            topic = self.mqtt_topic
        logging.info(f"Unsubscribing {topic}")
        self.client.unsubscribe(topic)
        logging.info(f"Unsubscribed {topic}")
        
    def disconnect(self):
        """
        Disconnect from the MQTT broker
        """
        logging.info("Disconnecting from MQTT broker...")
        self.disconnect_flag = True
        self.client.disconnect()
        logging.info("Disconnected from MQTT broker...")

    def on_unsubscribe(self, client, userdata, mid):
        """
        Callback function called when the client successfully unsubscribes from a topic
        """
        logging.info(f"Unsubscribed from topic with message id {str(mid)}")

    def on_subscribe(self, client, userdata, mid, granted_qos):
        """
        Callback function called when the client successfully subscribes to a topic
        """
        logging.info(f"Subscribed to topic with message id {str(mid)} and QoS {str(granted_qos)}")
    
    def publish(self, topic=None, message=None, qos=0):
        """
        Publish a message to the MQTT broker
        :param message: The message to publish
        """
        if topic is None:
            topic = self.mqtt_topic
        if message is None:
            message = "Warning: You have not specified the message to publish. Check out the Bridge class!"
        logging.info(f"Publishing messagethe message ('{message}') to topic {topic}")
        self.client.publish(topic, message, qos)
        
    def hook(self):
        """
        Gracefully shut down the MQTT client
        """
        self.unsubscribe()
        self.disconnect()
        logging.info("Shutting down...")

    def get_timeout(self):
        """
        Get the amount of time elapsed since the connection attempt started
        """
        return self.timeout
    