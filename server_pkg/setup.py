from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'server_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='won',
    maintainer_email='dwlee@k-loadrunner.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'service_server = server_pkg.service_server:main'
        'moving_client = server_pkg.moving_server:main'
        ],
    },
)
