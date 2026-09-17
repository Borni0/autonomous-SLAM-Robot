from setuptools import setup

package_name = 'rover_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/nav2_params.yaml',
            'config/amcl.yaml',
            'config/slam_toolbox.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/navigation.launch.py',
            'launch/mapping.launch.py',
        ]),
        # Empty placeholder map so the default `world:=` arg resolves
        # before the user has saved a real map.
        ('share/' + package_name + '/maps', [
            'maps/empty.yaml',
            'maps/empty.pgm',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rover Maintainer',
    maintainer_email='dev@example.com',
    description='Nav2 / AMCL / SLAM Toolbox configuration for the rover.',
    license='Apache-2.0',
)