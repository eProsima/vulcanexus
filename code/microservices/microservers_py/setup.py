from setuptools import setup

package_name = 'microservers_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    maintainer='jparisu',
    maintainer_email='javierparis@eprosima.com',
    description='Package implementing micro services for Vulcanexus tutorial',
    zip_safe=True,
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server_addition = microservers_py.microservice_server_addition:main',
            'server_subtraction = microservers_py.microservice_server_subtraction:main',
        ],
    },
)
