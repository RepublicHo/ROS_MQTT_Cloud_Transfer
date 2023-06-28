#!/usr/bin/env python


from bridge import Bridge
import rospy


# from mqtt to ros
# 1. connecting to an MQTT broker
# 2. subscribing to MQTT topics
# 3. processing incoming MQTT messages.


class FromMqttBridge(Bridge):
    
    def msg_process(self, msg):
        """
        Process incoming MQTT messages
        """
        msg_topic = msg.topic.split("/")
        if(msg_topic[-1] == "imu"):
            '''
            TODO: To modify
            extract the topic name from the MQTT message and
            split the payload into a list of strings. 
            '''
            topic_name = msg_topic[0].replace(" ", "_")
            msg_list = msg.payload.split(";")
        else:
            print(msg.topic + " is not a supported topic")



def main():
    rospy.init_node("mqtt_to_ros_processor", anonymous=True)
    test_sub2 = FromMqttBridge("MQTT_Test_Topic")
    rospy.on_shutdown(test_sub2.hook)

    while not rospy.is_shutdown():
        test_sub2.looping()


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