from setuptools import setup, find_packages

package_name = 'data_collector'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yiwei Wang',
    maintainer_email='yiwei03.wang@tum.de',
    description='ROS2 data collection package with timestamp alignment',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'data_collector_node = data_collector.data_collector_node:main',
        ],
    },
)