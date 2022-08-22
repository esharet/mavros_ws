from setuptools import setup
from glob import glob
import os

package_name = 'mavrover'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['launch','setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='eshaert@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_mavros = mavrover.main_mavros:main',
            'rc_over_pub = mavrover.rc_override_pub:main',
            'main_pymavlink = mavrover.main_pymavlink:main',
            'gazebo2topics = mavrover.gazebo2topics:main',
            'mavlink_msgs = mavrover.mavlink_msgs:main'
        ],
    },
)
