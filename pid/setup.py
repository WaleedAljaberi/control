import os
from glob import glob
from setuptools import setup

package_name = 'pid'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='waleed',
    maintainer_email='waleed@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = pid.pid_controller:main',
                'listener = pid.pid_controller2:main',            
                'control = pid.control:main',
                'plot = pid.plot:main',    
        ],
    },
)
