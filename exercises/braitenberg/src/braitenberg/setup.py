from setuptools import find_packages, setup
from glob import glob

package_name = 'braitenberg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # include models directory
        ('share/' + package_name + '/models', glob('models/*')),
        # include config directory
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jan Albrecht',
    maintainer_email='j_albr16@uni-muenster.de',
    description='Breitenberg project for ASMR course',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
