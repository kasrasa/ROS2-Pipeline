from setuptools import find_packages, setup

package_name = 'robot_state_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ksadatsh',
    maintainer_email='ksadatsh@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'state_publisher = robot_state_pipeline.state_publisher:main',
            'state_monitor = robot_state_pipeline.state_monitor:main',
            'move_action_server = robot_state_pipeline.move_action_server:main',
            'camera_publisher = robot_state_pipeline.camera_publisher:main',
            'plc_publisher = robot_state_pipeline.plc_publisher:main',
            'timestamp_synchronizer = robot_state_pipeline.timestamp_synchronizer:main',
        ],
    },
)
