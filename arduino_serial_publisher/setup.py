from setuptools import find_packages, setup

package_name = 'arduino_serial_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tanayrs',
    maintainer_email='tanay@tanayrs.com',
    description='ROS 2 package for controlling an Arduino robot using published pose data.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'key_move_serial = arduino_serial_publisher.key_move_serial:main',
            'key_move_print = arduino_serial_publisher.key_move_print:main',
            'publish_pose = arduino_serial_publisher.publish_pose:main',
        ],
    },
)
