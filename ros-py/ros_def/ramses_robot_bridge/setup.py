from setuptools import setup
from glob import glob

package_name = 'ramses_robot_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 serial bridge for Ramses robot arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ramses_robot_bridge = ramses_robot_bridge.ramses_robot_bridge:main',
        ],
    },
)
