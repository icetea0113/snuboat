from setuptools import find_packages, setup

package_name = 'snunav_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
                'mission_director = snunav_pkg.mission_director:main',
        ],
    },
)
