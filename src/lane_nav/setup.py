from setuptools import setup
import os
from glob import glob

package_name = 'lane_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='walter-amador',
    maintainer_email='amadorwalter3418@gmail.com',
    description='Lane-following autonomous navigation for LIMO robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lane_detection_node  = lane_nav.lane_detection_node:main',
            'lane_controller_node = lane_nav.lane_controller_node:main',
            'behavior_manager_node = lane_nav.behavior_manager_node:main',
        ],
    },
)
