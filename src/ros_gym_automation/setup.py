from setuptools import setup
import os
from glob import glob

package_name = 'ros_gym_automation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', glob('msg/*.msg')),
        ('share/' + package_name + '/srv', glob('srv/*.srv')),
    ],
    install_requires=[
        'setuptools',
        'gymnasium',
        'numpy',
        'jinja2',
    ],
    zip_safe=True,
    maintainer='Bedirhan Sen',
    maintainer_email='bdrhnsen@gmail.com',
    description='A package for automating the creation of ROS2 nodes from OpenAI Gym environments',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = ros_gym_automation.nodes.test_node:main',
        ],
    },
) 