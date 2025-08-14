from glob import glob 
import os 
from setuptools import find_packages, setup

package_name = 'snunav_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/'+package_name,["package.xml"]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools', 'snumsg_pkg'],
    zip_safe=True,
    maintainer='jyseo',
    maintainer_email='mouse89077@snu.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_interface = snunav_pkg.motor_interface:main',
            'navigation = snunav_pkg.navigation:main',
            'mission_director = snunav_pkg.mission_director:main',
            'controller = snunav_pkg.controller:main',
            'sils = snunav_pkg.sils:main',
            'udp_receiver = snunav_pkg.udp_receiver:main',
            'rqt = snunav_pkg.rqt:main',
        ],
    },
)
