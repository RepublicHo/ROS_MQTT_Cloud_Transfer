import paho.mqtt.client as mqtt
import time
import config as CONFIG

# Define a class to manage the MQTT client and callbacks
class StatusChecker:
    def __init__(self):
        self.client = mqtt.Client()
        self.dict = {'status_received': False, 'command_response_received': False}
        self.connected = False

        # Set up MQTT client callbacks
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected to MQTT broker with result code {rc}")
        
        if rc == 0:
            self.connected = True
            # Subscribe to the device's status topic    
            client.subscribe("iot_device/status")
            print(1)
        else:
            self.connected = False

    def on_message(self, client, userdata, msg):
        print(f"Received message on topic {msg.topic}: {msg.payload.decode()}")
        if msg.topic == "iot_device/status":
            # Set flag indicating that a status update has been received
            self.dict['status_received'] = True
            print("status received here")
            client.publish("iot_device/command", "status_check")
            client.unsubscribe("iot_device/status")
            client.subscribe("iot_device/command_response")
            print("unsubscribe ot_device/command")
        elif msg.topic == "iot_device/command_response":
            # Set flag indicating that the device is responding to commands
            self.dict['command_response_received'] = True
            print("!!!great, response received here")
            client.unsubscribe("iot_device/command_response")
            print("unsubscribe response")

    def connect(self):
        # Connect to MQTT broker and start client loop
        self.client.connect(CONFIG.CONNECTION.BROKER, CONFIG.CONNECTION.PORT, 60)
        self.client.loop_start()

        # Wait for connection to be established
        while not self.connected:
            print("trying to reconnect to the broker :(")
            time.sleep(1)

    # def disconnect(self):
    #     # Stop client loop and disconnect from MQTT broker
    #     self.client.loop_stop()
    #     self.client.disconnect()

    def send_command(self):
        # Send command to device to verify connectivity
        
        print("Sending command to iot_device/command")
        self.client.publish("iot_device/command", "status_check")
        time.sleep(1)

    def check_device_status(self, timeout=20.0):
        # Wait for timeout seconds or until a status update and command response are received
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            if self.dict['status_received'] and self.dict['command_response_received']:
                break
            time.sleep(1)

        # Return boolean value indicating device verification status
        return self.dict['status_received'] and self.dict['command_response_received']

    def get_device_status(self, timeout=20.0):
        # Connect to MQTT broker and subscribe to topics
        self.connect()
        self.client.message_callback_add("iot_device/status", self.on_message)
        self.client.message_callback_add("iot_device/command_response", self.on_message)

        # Wait for status update and send command to device
        time_wait = time.time() + timeout   # Loop for 5 seconds
        while time.time() < time_wait:
            self.client.loop()
            if self.dict['status_received']:
                print("status_received")
                break
        
        if self.dict['status_received']:
            self.send_command()
            
            # Wait for command response and check device status
            verification_status = self.check_device_status(timeout)

            # Disconnect from MQTT broker and return device verification status
            # self.disconnect()
            self.client.loop_stop()
            return verification_status
        else:
            self.client.loop_stop()
            return False


# Test code
def main():
    max_attempts = 3
    delay = 1
    attempt = 0
    while attempt < max_attempts:
        print(f"attempt {attempt}\n---")
        try:
            cli = StatusChecker()
            print(cli.get_device_status())
            return
        except Exception as e:
            print() 
            print(f"Exception occurred ({str(e)}), retrying in {delay} seconds")
            time.sleep(delay)
            attempt += 1
            
    print(f"number of attempts is ({attempt})")
    
if __name__ == "__main__":
    main()