from setuptools import setup

package_name = 'ramses_ik'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='IK solver and keyboard teleop for Ramses arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ik_solver       = ramses_ik.ik_solver:main',
            'teleop_keyboard = ramses_ik.teleop_keyboard:main',
        ],
    },
)