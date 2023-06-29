# import paho.mqtt.client as mqtt
# import time
# import config as CONFIG


# def publish_loop():
#     client = mqtt.Client()
#     client.connect(CONFIG.CONNECTION.BROKER, CONFIG.CONNECTION.PORT, 60)
#     client.loop_start()
#     while True:
#         # Publish a message to the "example/topic" topic every 5 seconds
#         client.publish("iot_device/status", "connected")
#         print("daemon process publishes to iot_device/status indicating it's there")
#         time.sleep(2)


# publish_loop()