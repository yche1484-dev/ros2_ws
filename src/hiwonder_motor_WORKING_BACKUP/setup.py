import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hiwonder_motor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include the launch directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pitest3',
    maintainer_email='todo@todo.com',
    description='Hiwonder Motor Control and Encoder Fusion Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hiwonder_encoder_node = hiwonder_motor.hiwonder_encoder_node:main',
            'imu_encoder_fusion_node = hiwonder_motor.imu_encoder_fusion_node:main',
            'omni_keyboard_node = hiwonder_motor.omni_keyboard_node:main',
            'omni_motor_node = hiwonder_motor.omni_motor_node:main',

            'hiwonder_motor_node = hiwonder_motor.hiwonder_motor_node:main',
            'mecanum_odometry_node = hiwonder_motor.mecanum_odometry_node:main',
        ],
    },
)
