#!/usr/bin/env python

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from time import sleep
import paho.mqtt.client as mqtt
from ros_mqtt_bridge import MQTTToROS
import struct
import rospy
from sensor_msgs.msg import PointCloud2

'''
Procedure
1. Bridging
2. Publish/Subscribe
'''

# Initialize a counter variable to keep track of the file index
file_index = 0
num_index = 0
sum = 0

# @deprecated
# This function is called when the client connects to the MQTT broker
def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker with result code " + str(rc))
    # Subscribe to the topic of interest
    client.subscribe("ABC", qos=2)

# @deprecated
# This function is called when a message is received on the subscribed topic
def on_message(client, userdata, msg):
    global file_index, num_index, sum
    print("Received message on topic " + msg.topic + " with payload size " + str(len(msg.payload)))

    # Unpack the binary message into a list of floats
    cloud_points_flat = struct.unpack('<%sf' % (len(msg.payload) // 4), msg.payload)

    # Save the cloud_points_flat data to a file with an index in the filename
    filename = 'cloud_sub_%d.txt' % file_index
    with open(filename, 'w') as f:
        for point in cloud_points_flat:
            sum += point
            f.write(str(num_index) + " " + str(point) + " " + str(sum) + '\n')
            num_index += 1

    # Increment the file index
    file_index += 1

    # Convert the list of floats to a list of tuples representing points
    # cloud_points = [(cloud_points_flat[i], cloud_points_flat[i+1], cloud_points_flat[i+2]) for i in range(0, len(cloud_points_flat), 3)]

    # Create a PointCloud2 message from the list of points
    # header = Header(frame_id="cloud_frame")
    


# def launch_mqtt_to_ros_bridge():
#     # create MQTTToROS instance with specific topics and message types
#     mqtt_to_ros = MQTTToROS("/mqtt/test/std_msgs_string", "/ros/test/std_msgs_string", "std_msgs/String")
#     print("start mqtt_to_ros_bridge.")
#     mqtt_to_ros.start()


if __name__ == '__main__':
    client = mqtt.Client()
    client.connect("121.41.94.38", 1883, 60)
    # Set the callback function for incoming messages
    client.on_connect = on_connect
    client.on_message = on_message

    client.loop_forever()

    # Start the MQTT loop to handle incoming messages
    # # Launch a bridge from ros(IoT device) to mqtt(cloud)
    # launch_mqtt_to_ros_bridge()