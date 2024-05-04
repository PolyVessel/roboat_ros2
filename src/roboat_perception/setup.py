from setuptools import setup
from setuptools.command.develop import develop
from setuptools.command.install import install
from subprocess import check_call
import os
import sys

package_name = 'roboat_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f"{package_name}/data"],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roboat',
    maintainer_email='<>',
    description='Roboats Sensors and Perception',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps = roboat_perception.gps:main',
            'radio = roboat_perception.radio:main',
            'recorder = roboat_perception.recorder:main'
        ],
    }
)
