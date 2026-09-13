from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'mos_coordinator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bolongli',
    maintainer_email='bli8@wpi.edu',
    description='MOS Coordinator',
    license='TODO',
    entry_points={
        'console_scripts': [
            'coordinator_node = mos_coordinator.coordinator_node:main',
            'simulator_node = mos_coordinator.simulator_node:main',
            'dummy_chassis_node = mos_coordinator.dummy_chassis_node:main',
            'dummy_arm_node = mos_coordinator.dummy_arm_node:main',
            'dummy_3d_object_node = mos_coordinator.dummy_3d_object_node:main',
            'dummy_cart_fullness_node = mos_coordinator.dummy_cart_fullness_node:main',
            'dummy_grasp_selector_node = mos_coordinator.dummy_grasp_selector_node:main',
            'flow_sm_test = mos_coordinator.flow_sm_test:main',
        ],
    },
)
