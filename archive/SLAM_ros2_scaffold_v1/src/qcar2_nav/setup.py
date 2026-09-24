from setuptools import setup
import os
from glob import glob

package_name = 'qcar2_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Antigravity',
    maintainer_email='antigravity@gemini.ai',
    description='QCar 2 SLAM and Navigation Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_goal = qcar2_nav.send_goal:main',
            'follow_waypoints = qcar2_nav.follow_waypoints:main',
            'verify_sensors = qcar2_nav.verify_sensors:main',
            'csi_hub = qcar2_nav.csi_hub:main',
        ],
    },
)
