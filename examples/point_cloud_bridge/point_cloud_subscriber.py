#!/usr/bin/env python
import rospy
from time import sleep

from ros_mqtt_bridge import MQTTToROS
from roslib import message
from sensor_msgs.msg import PointCloud2

'''
Procedure
1. Bridging
2. Publish/Subscribe
'''

def callback(data):
    rospy.loginfo("Received point cloud message with %d points" % len(data.data))

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('local_point_cloud_subscriber', anonymous=True)

    rospy.Subscriber("iot_data", PointCloud2, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def launch_mqtt_to_ros_bridge():
    # create MQTTToROS instance with specific topics and message types
    mqtt_to_ros = MQTTToROS("/mqtt/test/std_msgs_string", "/ros/test/std_msgs_string", "std_msgs/String")
    print("start mqtt_to_ros_bridge.")
    mqtt_to_ros.start()


if __name__ == '__main__':
    # Launch a bridge from ros(IoT device) to mqtt(cloud)
    launch_mqtt_to_ros_bridge()

    # ROS 
    listener()