from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'ds4_px4_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='uav_sim_ws user',
    maintainer_email='user@example.com',
    description='Mode 2 PX4 position control through MAVROS with a DS4.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'ds4_position_control = '
            'ds4_px4_control.ds4_position_control:main',
        ],
    },
)
