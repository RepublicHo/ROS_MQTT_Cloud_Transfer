#!/usr/bin/env python

from examples.point_cloud_bridge.bridge import bridge
import rospy
import struct
import paho.mqtt.publish as publish

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2


# from ROS to mqtt. 
# 1. connects to an MQTT broker
# 2. subscribes to a topic and 
# 3. immediately forwards the message to mqtt topic

# global variables for testing
msg_index = 0
num_index = 0
sum = 0

class ToMqttBridge(bridge.Bridge):

    def __init__(self, mqtt_topic, client_id = "test_mqtt_client", 
                 user_id="", password="", 
                 host="localhost", port="1883", keepalive=60, qos=0):
        """
        Constructor method for the ToMqttBridge class
        :param mqtt_topic: The topic to publish/subscribe to
        :param client_id: The ID of the client
        :param user_id: The user ID for the broker
        :param password: The password for the broker
        :param host: The hostname or IP address of the broker
        :param port: The port number of the broker
        :param keepalive: The keepalive interval for the client
        :param qos: The Quality of Service that determines the level of guarantee 
        for message delivery between MQTT client and broker. 
        """
        super().__init__(mqtt_topic, client_id, user_id, 
                         password, host, port, keepalive, qos)
        
    def callback(self, data):
        # global variables for testing
        global msg_index, num_index, sum

        # Convert the PointCloud2 message to a list of points. 
        cloud_points = list(point_cloud2.read_points(data, skip_nans=True, field_names = ("x", "y", "z")))

        # Convert each tuple in the list of points to a list of floats
        cloud_points_float = [[float(i) for i in point] for point in cloud_points]

        # Flatten the list of lists into a single list of floats
        cloud_points_flat = [coord for point in cloud_points_float for coord in point]

        # Test code: Save the binary message to a file with an index in the filename.
        filename = 'cloud_pub_%d.txt' % msg_index
        with open(filename, 'w') as f:
            for point in cloud_points_flat:
                sum += point
                f.write(str(num_index) + " " + str(point) + " " + str(sum) + '\n')
                num_index += 1

        # Increment the message index.
        msg_index += 1

        # Pack the list of floats into a binary string
        binary_msg = struct.pack('<%sf' % len(cloud_points_flat), *cloud_points_flat)

        # Publish the binary message to the MQTT topic
        self.publish(binary_msg)

        rospy.loginfo("Forwarder forwards point cloud message with payload size %d " % len(binary_msg))    


def main():
    # Initialize the ROS forwarder node, which can 
    # 1. Subscribe to a topic in ROS
    # 2. Immediately publish the point cloud message to the MQTT topic
    rospy.init_node('point_cloud_forwarder', anonymous=True)

    # mqtt topic named ABC is for testing 
    # Create an instance of the ToMqttBridge class
    # QoS is set as 2 to ensure the message is delivered exactly once, which brings more overhead. 
    bridge = ToMqttBridge(mqtt_topic="ABC", host="121.41.94.38", port=1883, qos=2)

    # Subscribe to the point cloud input topic and set the callback function
    rospy.Subscriber('/ouster/points', PointCloud2, bridge.callback)

    # Start the MQTT client's event loop
    bridge.looping()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        """
        If ROS interrupt exception is raised (e.g., if the user
        presses Ctrl-C to stop the program), the exception is
        caught and ignored. 
        """
        pass