# ROS_MQTT_Cloud_Transfer

暂时不考虑，这些在launcher里面的

python ros_to_mqtt.py -H <mqtt_broker_host> -P <mqtt_broker_port> -FT <ros_from_topic> -TT <mqtt_to_topic> -MT <message_type>

python mqtt_to_ros.py -H <mqtt_broker_host> -P <mqtt_broker_port> -FT <mqtt_from_topic> -TT <ros_to_topic> -MT <message_type>


正常用MQTT的command:

mosquitto_pub -h 121.41.94.38 -p 1883 -t ABC -m "Hello World!"
mosquitto_sub -h 121.41.94.38 -p 1883 -t ABC

如果在
## Requirements

- mosquitto(localhost:1883)

## Run

launch bridges
```
$ python std_msgs_string.py
```

launch subscribers
```
$ rostopic echo /ros/test/std_msgs_string
```

```
$ mosquitto_sub -d -t /mqtt/test/std_msgs_string -v
```

publish
```
$ rostopic pub -1 /ros/test/std_msgs_string std_msgs/String "data: 'test text from ros'"
```

```
$ mosquitto_pub -d -t /mqtt/test/std_msgs_string -m '{"data": "test text from mqtt"}'
```


![Alt text](image.png)