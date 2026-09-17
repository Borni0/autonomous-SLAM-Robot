from setuptools import setup

package_name = 'rover_compass'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/compass.yaml']),
        ('share/' + package_name + '/launch', ['launch/compass.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rover Maintainer',
    maintainer_email='dev@example.com',
    description='Compass / magnetometer ROS 2 node for the rover.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'compass_node = rover_compass.compass_node:main',
        ],
    },
)
