from setuptools import setup

package_name = 'rover_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/ekf.yaml']),
        ('share/' + package_name + '/launch', ['launch/ekf.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rover Maintainer',
    maintainer_email='dev@example.com',
    description='robot_localization EKF configuration for the rover.',
    license='Apache-2.0',
)
