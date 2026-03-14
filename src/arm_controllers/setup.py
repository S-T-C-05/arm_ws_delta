from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arm_controllers'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stc',
    maintainer_email='stc@todo.todo',
    description='Unified controller node for the robotic arm.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hardware_controller = arm_controllers.hardware_controller_node:main',
            'mock_controller = arm_controllers.mock_hardware_controller:main',
        ],
    },
)
