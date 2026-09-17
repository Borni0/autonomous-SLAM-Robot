from setuptools import setup

package_name = 'rover_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            ['config/diff_drive_params.yaml']),
        ('share/' + package_name + '/launch',
            ['launch/hardware.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Rover Maintainer',
    maintainer_email='dev@example.com',
    description='ESP32 serial bridge + dead-reckoning odometry for the rover.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_bridge        = rover_hardware.esp32_bridge:main',
            'diff_drive_odometry = rover_hardware.diff_drive_odometry:main',
            'soft_estop          = rover_hardware.soft_estop:main',
            'estop_cli           = rover_hardware.estop_cli:main',
        ],
    },
)
