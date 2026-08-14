import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bno085_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pitest3',
    maintainer_email='user@todo.todo',
    description='BNO085 ROS 2 Driver and Utilities',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bno085_node = bno085_driver.bno085_node:main',
            'imu_integrator_node = bno085_driver.imu_integrator_node:main',
        ],
    },
)
