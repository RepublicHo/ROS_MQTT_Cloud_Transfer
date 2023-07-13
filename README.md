# RosMqttBridge

中文版: [链接](./README_ZH.md)

This project is to establish bidirectional communication between the local host and the IoT device using MQTT bridges. For the IoT device we use, it is integrated with the ROS system and features a camera that can capture both point cloud and image data. By enabling the algorithm, the device can process this data and publish it to the local ROS topic. This device can also transmit data over a network. Due to the unknown public IPs of both the IoT device and the local host, as well as the large amount of data to be transferred and the unstable networking circumstances, we opted to to use the MQTT protocol to facilitate their long-range data transfer.

## Architecture Diagram
![Alt text](<architecture.png>)

## Getting Started

The project consists of three main components: an IoT device, a cloud platform, and a local host. The program are run in the localhost and device deparately. 

### Prerequisites

The things you need before installing the software.

* The device with ROS and other capturing capabilities.
* A reliable cloud server with public IP. It doesn't matter which cloud you choose (e.g., AWS EC2, Aliyun ECS or Tencent Cloud VM). Please put the IP in the Configuration classes in both device and server. 
* The localhost should have Linux environment and ROS installed.

### Installation


1. Have Eclipse Mosquitto properly installed in IoT device, cloud and localhost, following [link](https://mosquitto.org/download/), and configure the firewall in the cloud system to allow inbound traffic on port 1883. Check if it's binding to all available network interfaces by entering
   
```
$ netstat -an | grep 1883
tcp        0      0 0.0.0.0:1883            0.0.0.0:*               LISTEN  
```

## Test and Deploy

### IoT Device

In device, ensure that you have registered with the master node. Once you have registered, clone the files in the device folder to your IoT device and run the following command in your terminal:

```
$ python3 vibot_device.py
```

This command will start the program and subscribe to the command topic, waiting for commands from the server. It will be sending a regular heartbeat to indicate that the device is alive.

### Server Host

To get started in your localhost, clone the files in the host folder to your host machine and run the following command in your terminal:

```
$ python3 device_commander.py
```

This command will check if the device is alive. If the device is alive, you can enable the VIO algorithm and interact with the IoT device.

Please note that we have set an upper limit for the point cloud transfer in one transmission, and if you need to transfer more point cloud data, you can transmit the data again. You can also modify the upper limit as needed. And since the image data is too large to transmit in a single packet, we divided it into multiple packets for transmission. You can combine these packets in the data processors.

## Project status

This project is open for extension. Some suggestions:

* **Data Processors**: If you need to process additional data, such as saving, visualizing, or preprocessing, you can modify the data processors as needed.

* **Security**: To enhance security, you can set up authentication and authorization mechanisms to restrict access to the "status_check" and "status_ok" messages. You can also implement TLS protocol and set up usernames and passwords as appropriate.

* **Other Data Types**: If you need to transfer additional data types, such as focal lengths and locations, you can extend the project by adding similar forwarders, MQTT topics, and data processors with different implementations.

