from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mujoco_single_franka'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/mujoco_single_franka']),
        ('share/' + package_name, ['package.xml']),
        # Install all assets so they're accessible via ament_index at runtime
        (os.path.join('share', package_name, 'assets'),
            glob('assets/*.xml') + glob('assets/*.xml')),
        (os.path.join('share', package_name, 'assets/mesh'),
            glob('assets/mesh/*')),
        (os.path.join('share', package_name, 'assets/texture'),
            glob('assets/texture/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yiwei Wang',
    maintainer_email='yiwei03.wang@tum.de',
    description='',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'franka = mujoco_single_franka.franka:main',
        ],
    },
)
