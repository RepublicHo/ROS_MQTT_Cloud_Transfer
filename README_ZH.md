# RosMqttBridge

这个项目的目的是通过MQTT桥梁建立本地主机和IoT设备之间的双向通信。我们使用的IoT设备集成了ROS系统，并配备了一个摄像头，可以拍摄点云和图像数据。启用算法后，该设备可以处理这些数据并将其发布到本地ROS话题上。此设备还可以通过网络传输数据。由于IoT设备和本地主机的公网IP地址不确定，且需要传输大量数据，而且有的场景下网络环境不稳定，因此我们选择使用MQTT协议来实现它们的远距离数据传输。

## 架构图
![Alt text](<architecture.png>)
## 需要注意的点

1. 有时候Device有一个bug: Unable to register with master node. 好像无法通过kill杀进程重启roscore，只能reboot。但是有时候单次reboot也不行，只能多reboot几次，似乎这个ROS不如预期的稳定
2. 程序的安全性还需要加强。 Adversaries可以监听他们之间的通讯，甚至impersonate主机来获取设备所捕捉的数据，这是很危险的。
3. 极罕见情况，设备能够连接上网却无法正常push heartbeat，重启一下就好了。
 
Based on the code you provided, it seems that the image transfer and point cloud transfer are running in the main thread, which means that they will block the main thread until they finish. In general, it is a good practice to run long-running tasks, such as network I/O and file I/O, in a separate thread or process so that they do not block the main thread and cause the program to become unresponsive.

Therefore, it might be a good idea to start the image transfer and point cloud transfer in separate threads. You can use the threading module in Python to create threads. Here's an example of how you can start the image transfer and point cloud transfer in separate threads:

# Yes, there are more advanced systems that can be used to stop the data transfer in a more graceful and efficient way. Here are some examples:

# 应该用这个 Use a dedicated command topic: Instead of waiting for user input on the terminal, you can create a dedicated MQTT topic for sending control commands to the IoT device. The device can listen to this topic and stop the data transfer when it receives a stop command. This approach allows the user to stop the data transfer from any device that has access to the MQTT broker, not just the device running the program.
