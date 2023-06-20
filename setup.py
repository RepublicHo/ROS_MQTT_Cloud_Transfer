#!/usr/bin/env python
# coding: utf-8

from setuptools import setup, find_packages
from codecs import open
from os import path

here = path.abspath(path.dirname(__file__))

with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

with open(path.join(here, 'LICENSE'), encoding='utf-8') as f:
    _license = f.read()

setup(
    name='ros_mqtt_bridge',
    version='0.2',
    description='ros_mqtt_bridge provides bridge between ROS and MQTT(message serialized by JSON).',
    long_description=long_description,
    url='https://github.com/RepublicHo/ROS_MQTT_PointCloud_Transfer',
    author='Zhejun HE',
    author_email='anthony.zj.he@outlook.com',
    license=_license,
    packages=find_packages(exclude=["tests", "examples"]),
    install_requires=[
        "paho-mqtt==1.6/stable",
    ],
    test_suite="tests"
)