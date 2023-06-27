'''
Configuration class, which is used to store configuration 
settings or constants that are used throughout an application.
'''

class APP:
    APPLICATION = "Vibot Commander"
    AUTHOR = "Zhe-Jun HE"
    VERSION = "V1.0"

class CONNECTION:
    # I recommend to set static IP address on device
    IP = "192.168.1.150"

class DEFAULT:
    
    # Default volume from -80 to 18
    VOLUME = "-40"
    # Default input
    INPUT = "GAME"
    # Default sound mode
    SOUND_MODE = "MCH STEREO"