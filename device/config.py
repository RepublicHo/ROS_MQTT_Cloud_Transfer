'''
Configuration class, which is used to store configuration 
settings or constants that are used throughout an application.
'''

class APP:
    APPLICATION = "Vibot "
    AUTHOR = "Zhe-Jun HE"
    VERSION = "V1.0"

class CONNECTION:
    # Aliyun by Zhe
    BROKER_ADDRESS = "121.41.94.38" 

    TOPIC1 = "device/status"
    TOPIC2 = ""

class DEVICE:
    
    IP_ADDRESS = "192.168.1.73" #
    HTTP_PORT = "8000"
    