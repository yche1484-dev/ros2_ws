import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hcsr04_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pitest3',
    maintainer_email='pitest3@todo.todo',
    description='HC-SR04 ROS 2 Driver',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hcsr04_driver = hcsr04_pkg.hcsr04_driver:main',
            'ultrasonic_avoidance = hcsr04_pkg.ultrasonic_avoidance_node:main',
        ],
    },
)
