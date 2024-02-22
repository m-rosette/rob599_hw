import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rob599_hw2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marcus',
    maintainer_email='rosettem@oregonstate.edu',
    description='ROB599 HW2',
    license='BSD 3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_limiter = rob599_hw2.velocity_limiter:main',
            'twist_gen = rob599_hw2.twist_gen:main',
            'velocity_checker = rob599_hw2.velocity_checker:main',
            'braking_service_client = rob599_hw2.braking_service_client:main',
        ],
    },
)
