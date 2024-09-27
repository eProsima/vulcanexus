from setuptools import find_packages, setup

package_name = 'fastdds_cli'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eprosima',
    maintainer_email='juanlopez@eprosima.com',
    description='Fast DDS CLI',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'discovery_server = fastdds_cli.discovery_server:main',
            'shm = fastdds_cli.shm:main',
            'xml = fastdds_cli.xml:main'
        ],
    },
)
