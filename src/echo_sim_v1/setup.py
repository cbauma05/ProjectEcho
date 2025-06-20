import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'echo_sim_v1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'echo_sim_v1', 'launch'), glob('launch/*.py')),
        (os.path.join('share',package_name,'config'),glob('config/*.yaml')),        # Enable access to config directory
        (os.path.join('share',package_name,'worlds'),glob('world/*.world')),        # Enable access to world directory
        (os.path.join('share',package_name,'models'),glob('models/*.sdf')),         # Enable access to models directory
        (os.path.join('share',package_name,'rviz'),glob('rviz/*.rviz')),            # Enable access to rviz directory
        (os.path.join('share',package_name,'urdf'),glob('urdf/*.urdf')),            # Enable access to urdf directory
        (os.path.join('share',package_name,'meshes'),glob('meshes/*')),         # Enable access to meshes directory
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cameron',
    maintainer_email='cbauma05@uoguelph.ca',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
