from setuptools import setup
import os
from glob import glob

package_name = 'wavefront'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml') + glob('maps/*.pgm'))
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'loucura = wavefront.loucura:main',
        ],
    },
)