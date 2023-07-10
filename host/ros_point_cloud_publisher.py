# #!/usr/bin/env python
# from time import sleep
# from typing_extensions import deprecated
# import struct
# import rospy
# import paho.mqtt.publish as publish
# from sensor_msgs.msg import PointCloud
# from sensor_msgs import point_cloud2

# '''
# Procedure
# 1. Bridging
# 2. Publish/Subscribe
# '''

# msg_index = 0
# num_index = 0
# sum = 0

# @deprecated
# def callback(data):
#     global msg_index, num_index, sum
#     # This function is called every time a new point cloud message is received
#     # rospy.loginfo("Forwarder received point cloud message with %d points" % len(data.data))
    
#     # Convert the PointCloud message to a list of points.
    
#     cloud_points = list(point_cloud2.read_points(data, skip_nans=True, field_names = ("x", "y", "z")))

#     # Convert each tuple in the list of points to a list of floats
#     cloud_points_float = [[float(i) for i in point] for point in cloud_points]

#     # Flatten the list of lists into a single list of floats
#     cloud_points_flat = [coord for point in cloud_points_float for coord in point]

#     # Save the binary message to a file with an index in the filename.
#     filename = 'cloud_pub_%d.txt' % msg_index
#     with open(filename, 'w') as f:
#         for point in cloud_points_flat:
#             sum += point
#             f.write(str(num_index) + " " + str(point) + " " + str(sum) + '\n')
#             num_index += 1

#     # Pack the list of floats into a binary string
#     binary_msg = struct.pack('<%sf' % len(cloud_points_flat), *cloud_points_flat)

#     # Increment the message index.
#     msg_index += 1

#     rospy.loginfo("Forwarder forwards point cloud message with payload size %d " % len(binary_msg))    

#     # Publish the binary message to MQTT topic
#     publish.single("ABC", payload=binary_msg, hostname="121.41.94.38", qos=2)
    
# # def launch_ros_to_mqtt_bridge():
# #     # create ROSToMQTT instance with specific topics and message types
# #     ros_to_mqtt = ROSToMQTT("point_cloud_output", "iot_data", "sensor_msgs/PointCloud2")
# #     print("start ros_to_mqtt_bridge.")
# #     ros_to_mqtt.start()

# if __name__ == '__main__':

#     # # Launch a bridge from ros(IoT device) to mqtt(cloud)
#     # launch_ros_to_mqtt_bridge()

#     # Initialize the ROS node and publisher
#     rospy.init_node('point_cloud_forwarder', anonymous=True)
#     # pub = rospy.Publisher('iot_data', PointCloud2, queue_size=10)

#     # Subscribe to the point cloud input topic
#     rospy.Subscriber('/ouster/points', PointCloud2, callback)

#     # Spin the node to keep it running and wait for incoming messages
#     rospy.spin()

