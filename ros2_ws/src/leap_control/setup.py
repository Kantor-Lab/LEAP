from setuptools import setup
import os
from glob import glob

package_name = 'leap_control'
# launch_files = [os.path.join('launch', f) for f in os.listdir('launch') if f.endswith('.py')]
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Install resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        
        # # Install launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),

        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        # ('share/' + package_name + '/launch', launch_files),

        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atharvgo',
    maintainer_email='atharvgo@andrew.cmu.edu',
    description='Package to control physical and simulated Farmng Amiga robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amiga_control = leap_control.amiga_control:main',
            'cmdvel_relay = leap_control.cmdvel_relay:main',
            'imu_calibration = leap_control.imu_calibration:main',
            'imu_relay = leap_control.imu_relay:main',
            'ply_publisher = leap_control.ply_publisher:main',
        ],
    },
)
