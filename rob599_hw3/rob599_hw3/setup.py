import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rob599_hw3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'resource'), glob(os.path.join('resource', '*[.yaml,.pgm]'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marcus',
    maintainer_email='rosettem@oregonstate.edu',
    description='ROB599 Homework 3',
    license='BSD 3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'places = rob599_hw3.places:main',
            'goto_client = rob599_hw3.goto_action_client:main',
        ],
    },
)
