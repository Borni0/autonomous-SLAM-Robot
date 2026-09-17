from setuptools import setup

package_name = 'rover_lidar'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/ydlidar.yaml']),
        ('share/' + package_name + '/launch', ['launch/a3.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rover Maintainer',
    maintainer_email='dev@example.com',
    description='Thin wrapper around ydlidar_ros2_driver for the A3.',
    license='Apache-2.0',
    tests_require=['pytest'],
)
