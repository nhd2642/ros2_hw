from glob import glob

from setuptools import setup

package_name = 'terrain_perception'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nhd2642',
    maintainer_email='nhd2642@knu.ac.kr',
    description='Depth-camera terrain hazards for Nav2 costmaps',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['terrain_hazards = terrain_perception.terrain_node:main']},
)
