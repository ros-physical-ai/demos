from glob import glob
import os

from setuptools import setup

PACKAGE_NAME = 'pai_aic'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=[PACKAGE_NAME, f'{PACKAGE_NAME}.policies', f'{PACKAGE_NAME}.tests'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join('share', PACKAGE_NAME, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', PACKAGE_NAME, 'config'), glob('config/*.yaml')),
        (os.path.join('share', PACKAGE_NAME, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Franco Cipollone',
    maintainer_email='franco@example.com',
    description='Bridge demos Record/Train/Deploy pipeline to AIC cable-insertion scenario',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={},
)