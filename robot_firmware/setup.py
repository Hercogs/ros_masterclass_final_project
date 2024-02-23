from setuptools import setup
import os
from glob import glob

package_name = 'robot_firmware'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "main_node = robot_firmware.main_node:main",
            "table_detection = robot_firmware.table_detection:main",
            "tf2_listener = robot_firmware.tf2_listener:main",
            "action = robot_firmware.action:main",
            "test_nav2 = robot_firmware.test_nav2:main",
        ],
    },
)
