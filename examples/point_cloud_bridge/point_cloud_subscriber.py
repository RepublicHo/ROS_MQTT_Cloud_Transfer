#!/usr/bin/env python

from time import sleep
import paho.mqtt.client as mqtt
from ros_mqtt_bridge import MQTTToROS
import struct
from sensor_msgs.msg import PointCloud2

'''
Procedure
1. Bridging
2. Publish/Subscribe
'''


def listener(client, userdata, message):

    print("Received message on topic: ", message.topic)

    # Convert the binary message to na list of points
    cloud_points = struct.unpack("<%df" % (len(message.payload)/4), message.payload)

    # Create a PointCloud2 message
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "base_link"
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]
    cloud_msg = point_cloud2.create_cloud(header, fields, cloud_points)
    


# def launch_mqtt_to_ros_bridge():
#     # create MQTTToROS instance with specific topics and message types
#     mqtt_to_ros = MQTTToROS("/mqtt/test/std_msgs_string", "/ros/test/std_msgs_string", "std_msgs/String")
#     print("start mqtt_to_ros_bridge.")
#     mqtt_to_ros.start()


if __name__ == '__main__':
    client = mqtt.Client()
    client.connect("121.41.94.38")
    client.subscribe("ABC")
    # Set the callback function for incoming messages
    client.on_message = listener

    client.loop_forever()

    # Start the MQTT loop to handle incoming messages
    # # Launch a bridge from ros(IoT device) to mqtt(cloud)
    # launch_mqtt_to_ros_bridge()
