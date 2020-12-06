#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['lisa_mqtt'],
    package_dir={'': 'src'},
    install_requires=['paho-mqtt', 'inject', 'msgpack-python', 'json', 'transitions', 'soundfile']
)

setup(**d)
