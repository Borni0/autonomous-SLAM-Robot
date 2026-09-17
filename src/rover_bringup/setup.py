from setuptools import setup

package_name = 'rover_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/rover.launch.py',
            'launch/test_motors.launch.py',
            'launch/test_lidar.launch.py',
            'launch/test_odom.launch.py',
            'launch/teleop_twist.launch.py',
        ]),
        ('share/' + package_name + '/scripts', [
            'scripts/drive_once.sh',
        ]),
        ('share/' + package_name + '/rviz', ['rviz/rover.rviz']),
        ('share/' + package_name + '/config', ['config/bringup_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rover Maintainer',
    maintainer_email='dev@example.com',
    description='Top-level launch orchestration for the rover.',
    license='Apache-2.0',
)
