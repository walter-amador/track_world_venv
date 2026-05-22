from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'obst_avoid'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='walter-amador',
    maintainer_email='amadorwalter3418@gmail.com',
    description='Depth estimation benchmark and obstacle avoidance PoC for LIMO sim',
    license='MIT',
    entry_points={
        'console_scripts': [
            'depth_benchmark_node   = obst_avoid.depth_benchmark_node:main',
            'obstacle_avoidance_node = obst_avoid.obstacle_avoidance_node:main',
        ],
    },
)
