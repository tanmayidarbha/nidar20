from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'drone_pipeline'


setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='tanmayi_unix',
    maintainer_email='tanmayi_unix@example.com',

    description='NIDAR Barbarik and Rudra autonomous rescue pipeline',

    license='Apache-2.0',

    entry_points={
        'console_scripts': [

            # Barbarik
            'camera_node = drone_pipeline.camera_node:main',
            'detection_node = drone_pipeline.detection_node:main',
            'geotag_node = drone_pipeline.geotag_node:main',
            'csv_logger_node = drone_pipeline.csv_logger_node:main',
            'mission_control_node = drone_pipeline.mission_control_node:main',

            # Fusion
            'fusion_node = drone_pipeline.fusion_node:main',

            # Rudra
            'nav_delivery_node = drone_pipeline.nav_delivery_node:main',
            'payload_control_node = drone_pipeline.payload_control_node:main',
            'rfd_receiver_node = drone_pipeline.rfd_receiver_node:main',
            'rfd_sender_node = drone_pipeline.rfd_sender_node:main',
        ],
    },
)
