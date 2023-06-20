#!/usr/bin/env python
# coding: utf-8

from time import sleep
from multiprocessing import Process

from ros_mqtt_bridge import MQTTToROS, ROSToMQTT
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

# one version of compressing cloud point data is to convert it to numpy array. 

def callback(data):
    # This function is called every time a new point cloud message is received
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame.id = data.header.frame.id

    # Publish the received point cloud data on another topic
    pub.publish(PointCloud2(header=header, height=data.height, width=data.width, 
                            fields=data.fields, is_bigendian=data.is_bigendian, 
                            point_step=data.point_step, row_step=data.row_step, 
                            data=data.data, is_dense=data.is_dense))


def launch_ros_to_mqtt_bridge():
    ros_to_mqtt = ROSToMQTT("/ros/test/std_msgs_string", "/mqtt/test/std_msgs_string", "std_msgs/String")
    print("start ros_to_mqtt_bridge.")
    ros_to_mqtt.start()


def launch_mqtt_to_ros_bridge():
    mqtt_to_ros = MQTTToROS("/mqtt/test/std_msgs_string", "/ros/test/std_msgs_string", "std_msgs/String")
    print("start mqtt_to_ros_bridge.")
    mqtt_to_ros.start()


if __name__ == '__main__':

    # Initialize the ROS node and publisher
    rospy.init_node("point_cloud_forwarder", anonymous=True)
    pub = rospy.Publisher('point_cloud_output', PointCloud2, queue_size=10)

    # Subscribe to the point cloud input topic
    rospy.Subscriber('point_cloud_input', PointCloud2, callback)
    process_mqtt_to_ros = Process(target=launch_mqtt_to_ros_bridge)
    process_ros_to_mqtt = Process(target=launch_ros_to_mqtt_bridge)
    process_mqtt_to_ros.start()
    process_ros_to_mqtt.start()



    sleep(600)
    process_mqtt_to_ros.terminate()
    process_ros_to_mqtt.terminate()
    print("terminate bridges")
