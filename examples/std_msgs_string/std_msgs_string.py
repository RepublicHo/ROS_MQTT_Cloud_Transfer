#!/usr/bin/env python
# coding: utf-8

from time import sleep
from multiprocessing import Process

from ros_mqtt_bridge import MQTTToROS, ROSToMQTT
'''
    Launches two bridges (as two processes) that facilitate communication 
    between ROS-based systems and MQTT-based systems

'''

def launch_ros_to_mqtt_bridge():
    # create ROSToMQTT instance with specific topics and message types
    ros_to_mqtt = ROSToMQTT("/ros/test/std_msgs_string", "/mqtt/test/std_msgs_string", "std_msgs/String")
    print("start ros_to_mqtt_bridge.")
    ros_to_mqtt.start()


def launch_mqtt_to_ros_bridge():
    # create MQTTToROS instance with specific topics and message types
    mqtt_to_ros = MQTTToROS("/mqtt/test/std_msgs_string", "/ros/test/std_msgs_string", "std_msgs/String")
    print("start mqtt_to_ros_bridge.")
    mqtt_to_ros.start()


if __name__ == '__main__':
    # create two process instances to run two bridges concurrently
    process_mqtt_to_ros = Process(target=launch_mqtt_to_ros_bridge)
    process_ros_to_mqtt = Process(target=launch_ros_to_mqtt_bridge)
    process_mqtt_to_ros.start()
    process_ros_to_mqtt.start()
    # wait for 60 seconds before terminating processes
    sleep(60)
    print("terminate bridges")
    process_mqtt_to_ros.terminate()
    process_ros_to_mqtt.terminate()