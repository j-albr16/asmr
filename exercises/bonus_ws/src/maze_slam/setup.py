from setuptools import find_packages, setup
from glob import glob

package_name = 'maze_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/world', glob('world/*')),
        (f'share/{package_name}/rviz', glob('rviz/*')),
        (f'share/{package_name}/urdf', glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jan Albrecht',
    maintainer_email='jan.albrecht2000@gmail.com',
    description='Slam Assignment for the ASMR Tutorium',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
