from setuptools import find_packages, setup

package_name = 'cylinder_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cylinder_launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/cylinder.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Achyut Shrestha',
    maintainer_email='achyutros@gmail.com',
    description='Achyut Assignment 1',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cylinder_node = cylinder_detection.cylinder_node:main'
        ],
    },
)
