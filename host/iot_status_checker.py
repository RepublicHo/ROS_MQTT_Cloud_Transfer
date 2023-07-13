import paho.mqtt.client as mqtt
import time
import config as CONFIG
from bridge import Bridge

# Define a class to check the status of the IoT device
# Once unconnected or started, try to estblish three-way handshake
class StatusChecker(Bridge):
    def __init__(self, client_id = "client", 
                 user_id="", password="", 
                 host="localhost", port=1883, keepalive=60, qos=0): 
        
        
        
        # status check is composed of heartbeat check and response check. 
        # status_code is used for checking if the host can connect to the device through
        # 1. the host can sense the heartbeat of the device. 
        # 2. the host can verify the device, making sure it's not the malicious user. 
        # 0 for not connected, 1 for connected, 2 for bug
        self.status = {'heartbeat': False, 'response_received': False}
        self.status_code = 0 
        
        # constants for brokers
        self.DEVICE_HEARTBEAT = "/iot_device/heartbeat"
        self.STATUS_CHECK = "/iot_device/status_check"
        self.STATUS_RESPONSE = "/iot_device/status_response"
        
        super().__init__(self.DEVICE_HEARTBEAT, client_id, user_id, 
                         password, host, port, keepalive, qos)

    def on_connect(self, client, userdata, flags, rc):
        
        print(f"Connected to MQTT broker with result code {rc}")
        
        # Subscribe to the device's heartbeat topic    
        self.subscribe(self.DEVICE_HEARTBEAT)

    def msg_process(self, msg):
        
        print(f"Received message on topic {msg.topic}: {msg.payload.decode()}")
        
        if msg.topic == self.DEVICE_HEARTBEAT:
            
            # Set flag indicating that a heartbeat has been received
            self.status['heartbeat'] = True
            print(":) The heartbeat of vibot received")
            self.unsubscribe(self.DEVICE_HEARTBEAT)
            self.subscribe(self.STATUS_RESPONSE)
            # it can be replaced by passwords or others related to cryptography
            # self.publish(topic=self.STATUS_CHECK, message="status_check", qos=2)

            
        elif msg.topic == self.STATUS_RESPONSE:
            # Set flag indicating that the device is responding to commands
            self.status['response_received'] = True
            print("!!!great, response received here")
            self.unsubscribe(self.STATUS_RESPONSE)


    # def connect(self):
    #     # Connect to MQTT broker and start client loop
    #     self.client.connect(CONFIG.CONNECTION.BROKER, CONFIG.CONNECTION.PORT, 60)
    #     self.client.loop_start()

    #     # Wait for connection to be established
    #     while not self.connected:
    #         print("trying to reconnect to the broker :(")
    #         time.sleep(1)

    # def disconnect(self):
    #     # Stop client loop and disconnect from MQTT broker
    #     self.client.loop_stop()
    #     self.client.disconnect()

    def send_command(self):
        # Send command to device to verify connectivity
        
        self.publish(topic=self.STATUS_CHECK, message="status_check", qos=2)
        time.sleep(1)

    def check_device_status(self, timeout=20.0):
        # Wait for timeout seconds or until a status update and command response are received
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            if self.status['heartbeat'] and self.status['response_received']:
                break

        # Return boolean value indicating device verification status
        return self.status['heartbeat'] and self.status['response_received']


    def get_device_status(self, timeout=20.0):
        # Connect to MQTT broker and subscribe to topics
        self.connect()
        self.client.message_callback_add(self.DEVICE_HEARTBEAT, self.on_message)
        self.client.message_callback_add(self.STATUS_RESPONSE, self.on_message)

        # Wait for status update and send command to device
        time_wait = time.time() + timeout   # Loop for 5 seconds


        self.client.loop_start()
        while time.time() < time_wait:
        
            if self.status['heartbeat'] == True:
                print("heartbeat detected")
                break
            time.sleep(1)

        if self.status['heartbeat']:
            self.send_command()
            print(":) We have verify the IoT device. Let's move on!")
            # Wait for command response and check device status
            self.status_code = self.check_device_status(timeout)

            # Disconnect from MQTT broker and return device verification status
            # self.disconnect()
            self.client.loop_stop()
            print(f"status code: {self.status_code}")
            return self.status_code
                
        else:
            self.client.loop_stop()
            print(":( Please be advised that the vibot may fail to connect to the Internet.")
            print("1. Try to rerun the Vibot and check its network. ")
            print("2. Try to rerun this program. ")
            return 0
                
        
    

# Test code
def main():
    max_attempts = 3
    delay = 1
    attempt = 0
    
    # print(f"attempt {attempt}\n---")
    
    cli = StatusChecker(client_id = "status_checker", host="43.133.159.102", port=1883)
    print(cli.get_device_status())
        
    # except Exception as e:
    #     print() 
    #     print(f"Exception occurred ({str(e)}), retrying in {delay} seconds")
    #     time.sleep(delay)
            
            
    # print(f"number of attempts is ({attempt})")
    
if __name__ == "__main__":
    main()