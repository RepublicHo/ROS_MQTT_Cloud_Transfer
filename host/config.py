'''
Configuration class, which is used to store configuration 
settings or constants that are used throughout an application.
'''

class APP:
    APPLICATION = "Vibot Commander"
    AUTHOR = "Zhe-Jun HE"
    VERSION = "V1.0"

class CONNECTION:
    # Aliyun by Zhe
    BROKER = "43.133.159.102" 
    PORT = 1883
    TOPIC1 = "device/status"
    TOPIC2 = ""

class DEVICE:
    
    IP = "192.168.1.73" #
    HTTP_PORT = "8000"
    