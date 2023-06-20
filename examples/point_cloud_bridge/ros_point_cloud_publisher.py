#!/usr/bin/env python
from time import sleep

from ros_mqtt_bridge import ROSToMQTT

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

'''
Procedure
1. Bridging
2. Publish/Subscribe
'''

def callback(data):
    rospy.loginfo("Forwarder received point cloud message with %d points" % len(data.data))
    # This function is called every time a new point cloud message is received
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = data.header.frame_id

    # Publish the received point cloud data on another topic
    pub.publish(PointCloud2(header=header, height=data.height, width=data.width, fields=data.fields,
                            is_bigendian=data.is_bigendian, point_step=data.point_step,
                            row_step=data.row_step, data=data.data, is_dense=data.is_dense))
    
def launch_ros_to_mqtt_bridge():
    # create ROSToMQTT instance with specific topics and message types
    ros_to_mqtt = ROSToMQTT("point_cloud_output", "iot_data", "sensor_msgs/PointCloud2")
    print("start ros_to_mqtt_bridge.")
    ros_to_mqtt.start()

if __name__ == '__main__':
    # Launch a bridge from ros(IoT device) to mqtt(cloud)
    launch_ros_to_mqtt_bridge()

    # Initialize the ROS node and publisher
    rospy.init_node('point_cloud_forwarder', anonymous=True)
    pub = rospy.Publisher('iot_data', PointCloud2, queue_size=10)

    # Subscribe to the point cloud input topic
    rospy.Subscriber('/ouster/points', PointCloud2, callback)

    # Spin the node to keep it running and wait for incoming messages
    rospy.spin()