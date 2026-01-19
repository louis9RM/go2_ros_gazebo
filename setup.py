import os
from glob import glob
from setuptools import setup

package_name = 'go2_ros_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.*')),
        (os.path.join('share', package_name, 'dae'), glob('dae/*.dae')),
        (os.path.join('share', package_name, 'checkpoints'), glob('checkpoints/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ever',
    maintainer_email='louis9ramos@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go2_rl_driver = go2_ros_gazebo.go2_rl_driver:main',
        ],
    },
)
