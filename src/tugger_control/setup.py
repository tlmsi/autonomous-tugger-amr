from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tugger_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='osama',
    maintainer_email='osama@example.com',
    description='Vehicle control for the autonomous 4WS tugger AMR',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'four_ws_controller = tugger_control.four_ws_controller:main',
            'four_ws_odometry = tugger_control.four_ws_odometry:main',
            'imu_orientation_plotter = tugger_control.imu_orientation_plotter:main',
        ],
    },
)
