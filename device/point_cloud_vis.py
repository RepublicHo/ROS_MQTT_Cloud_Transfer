# #!/usr/bin/env python

# import rospy
# from sensor_msgs.msg import PointCloud

# # Import the visualization_msgs/MarkerArray message type
# from visualization_msgs.msg import Marker, MarkerArray

# def callback(data):
#     # Create a new MarkerArray message
#     marker_array = MarkerArray()

#     # Set the header of the MarkerArray message
#     # marker_array.header = data.header

#     # Loop through each point in the point cloud
#     for p in data.points:
#         # Create a new Marker message for each point
#         marker = Marker()

#         # Set the type of the marker to a sphere
#         marker.type = Marker.SPHERE

#         # Set the position of the marker to the position of the point
#         marker.pose.position.x = p.x
#         marker.pose.position.y = p.y
#         marker.pose.position.z = p.z

#         # Set the scale of the marker to 0.1 meters in all directions
#         marker.scale.x = 0.1
#         marker.scale.y = 0.1
#         marker.scale.z = 0.1

#         # Set the color of the marker to green
#         marker.color.r = 0.0
#         marker.color.g = 1.0
#         marker.color.b = 0.0
#         marker.color.a = 1.0

#         # Append the marker to the MarkerArray message
#         marker_array.markers.append(marker)

#     # Publish the MarkerArray message
#     pub.publish(marker_array)

# if __name__ == '__main__':
#     # Initialize the ROS node
#     rospy.init_node('point_cloud_visualizer')

#     # Create a publisher for the MarkerArray topic
#     pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)

#     # Create a subscriber for the PointCloud topic
#     sub = rospy.Subscriber('/PR_BE/point_cloud', PointCloud, callback)

#     # Spin the node to receive messages
#     rospy.spin()