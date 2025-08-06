from setuptools import find_packages, setup
import glob
import os

package_name = 'submarine_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['submarine_pkg/calibration.json']),
        (os.path.join('share', package_name), glob.glob(os.path.join(package_name, '*.json'))),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'sensor_msgs', 'geometry_msgs'],
    zip_safe=True,
    maintainer='rpi',
    maintainer_email='rpi@todo.todo',
    description='package for submarine/boat control (WIP)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'imu_publisher = submarine_pkg.imu_publisher:main',
            'websocket_bridge = submarine_pkg.websocket_bridge:main',
        ],
    },
)
