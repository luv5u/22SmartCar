from setuptools import find_packages, setup

package_name = 'sensor_chain'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luv5u',
    maintainer_email='rkdy030628@outlook.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'alarm_server = sensor_chain.alarm_server:main',
            'node_c_alarm = sensor_chain.node_c_alarm:main',
            'node_b_filter = sensor_chain.node_b_filter:main',
            'node_a_sensor = sensor_chain.node_a_sensor:main'
        ],
    },
)
