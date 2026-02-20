import os
from glob import glob
from setuptools import setup

package_name = 'crew_protocol'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chris',
    maintainer_email='chris@crew.dev',
    description='CREW Emergency Robot Protocol',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'broadcast = crew_protocol.broadcast:main',
            'robot = crew_protocol.robot:main',
            'coordinator = crew_protocol.coordinator:main',
            'multi_robot = crew_protocol.multi_robot:main',
        ],
    },
)
