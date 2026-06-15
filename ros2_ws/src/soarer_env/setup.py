import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'soarer_env'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='px4_sitl',
    maintainer_email='px4_sitl@todo.todo',
    description='Environmental and sensing nodes for the soarer swarm simulation',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thermal_field_node = soarer_env.thermal_field_node:main',
            'ground_event_node = soarer_env.ground_event_node:main',
            'fov_detection_node = soarer_env.fov_detection_node:main',
            'battery_estimator_node = soarer_env.battery_estimator_node:main',
            'fsm_node = soarer_env.fsm_node:main',
            'metrics_collector_node = soarer_env.metrics_collector_node:main',
        ],
    },
)
