import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ros_introduction'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
        (os.path.join('share', package_name, 'parameters'), glob('parameters/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sguajardo799',
    maintainer_email='sguajardo@protonmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_node = ros_introduction.simple_node:main',
	        'temp_publisher = ros_introduction.temperature_publisher:main',
            'temp_subscriber = ros_introduction.temperature_subscriber:main',
            'actuator_service = ros_introduction.actuator_service:main',
            'actuator_client = ros_introduction.actuator_client:main',
        ],
    },
)
