from glob import glob

from setuptools import find_packages, setup

package_name = 'comm_relay'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test', 'tools']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nhd2642',
    maintainer_email='nhd2642@knu.ac.kr',
    description='Husky mission and driving, A1 follow / exploration executor / comm return / survivor report',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'clock_throttle = comm_relay.clock_throttle:main',
            'husky_mission = comm_relay.husky_mission:main',
            'husky_driver = comm_relay.husky_driver:main',
            'a1_follower = comm_relay.a1_follower:main',
            'return_relay = comm_relay.return_relay:main',
            'cmd_vel_arbiter = comm_relay.cmd_vel_arbiter:main',
            'frontier_comm_filter = comm_relay.frontier_comm_filter:main',
            'nav_executor = comm_relay.nav_executor:main',
            'survivor_detector = comm_relay.survivor_detector:main',
        ],
    },
)
