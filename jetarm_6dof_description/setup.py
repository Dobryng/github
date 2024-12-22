from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'jetarm_6dof_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
        (os.path.join('share',package_name,'urdf'),glob(os.path.join('urdf','*.*'))),
        (os.path.join('share',package_name,'meshes/gripper'),glob(os.path.join('meshes/gripper','*.*'))),
        (os.path.join('share',package_name,'meshes/jetarm_6dof'),glob(os.path.join('meshes/jetarm_6dof','*.*'))),        
        (os.path.join('share',package_name,'meshes'),glob(os.path.join('meshes','*.*'))),
        (os.path.join('share',package_name,'rviz'),glob(os.path.join('rviz','*.rviz*'))),
        (os.path.join('share',package_name,'gazebo'),glob(os.path.join('gazebo','*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orin',
    maintainer_email='orin@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)