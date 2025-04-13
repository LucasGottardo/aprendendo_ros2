from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'wavefront'

# Caminho absoluto para a pasta do pacote
package_dir = os.path.abspath(os.path.dirname(__file__))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Caminho corrigido para os arquivos do mapa
        (os.path.join('share', package_name, 'maps'), 
         glob(os.path.join(package_dir, 'maps', '*')))
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'loucura = wavefront.loucura:main',
        ],
    },
)