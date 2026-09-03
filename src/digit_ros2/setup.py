from setuptools import setup

package_name = 'digit_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/digit.launch.py',
            'launch/digit_wsl2.launch.py',
            'launch/digit_robot.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO: your name',
    maintainer_email='landradepro@alumni.unav.es',
    description='ROS2 driver node for the DIGIT tactile sensor',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_publisher = digit_ros2.camera_publisher_node:main',
            'contact_detector = digit_ros2.contact_detector_node:main',
            'pressure_estimator = digit_ros2.pressure_estimator_node:main',
            'depth_reconstructor = digit_ros2.depth_reconstructor_node:main',
        ],
    },
)
