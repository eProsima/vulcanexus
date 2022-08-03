from setuptools import setup

package_name = 'clients_py'

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
            'client_addition = clients_py.microservice_client_addition:main',
            'client_subtraction = clients_py.microservice_client_subtraction:main',
        ],
    },
)
