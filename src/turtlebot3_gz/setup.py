from setuptools import find_packages, setup

from glob import glob #added
import os #added

package_name = 'turtlebot3_gz'

#====================================================================================================
#====================================================================================================
# The 'setup.py' file is essential for building and installing a ROS 2 package using 'colcon build'. 
# It ensures that all necessary files and directories are correctly set up for the package to run.
#
# This file mainly serves the following purposes:
# 
# 1. Defines the package metadata such as name, version, and description.
# 2. Specifies dependencies required for the package.
# 3. Configures directories that need to be accessed (e.g., launch files, scripts).
# 4. Registers Python nodes as executables using the 'entry_points' script.
#
# Proper configuration of 'setup.py' ensures that commands like 'ros2 launch' and 'ros2 run' work 
# correctly by making scripts and launch files accessible within the ROS 2 environment.
#====================================================================================================
#====================================================================================================

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob('launch/*.launch.py')),   # Enable access to launch directory
        (os.path.join('share',package_name,'config'),glob('config/*.yaml')),        # Enable access to config directory
        (os.path.join('share',package_name,'worlds'),glob('world/*.world')),        # Enable access to world directory
        (os.path.join('share',package_name,'models'),glob('models/*.sdf')),         # Enable access to models directory
        (os.path.join('share',package_name,'rviz'),glob('rviz/*.rviz')),            # Enable access to rviz directory
        (os.path.join('share',package_name,'urdf'),glob('urdf/*.urdf')),            # Enable access to urdf directory
        (os.path.join('share',package_name,'meshes'),glob('meshes/*.dae')),         # Enable access to meshes directory
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matthew Salazar',
    maintainer_email='msalaz03@uoguelph.ca',
    description='RVIZ Introduction',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
