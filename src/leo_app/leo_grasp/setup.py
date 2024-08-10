from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'leo_grasp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob(os.path.join('launch','*launch.py'))),
        (os.path.join('share', package_name,'rviz'), glob(os.path.join('rviz','*.rviz'))),
        (os.path.join('share', package_name,'scripts'), glob(os.path.join('scripts','*.sh'))),
        (os.path.join('share', package_name,'param'), glob(os.path.join('param','*.txt'))),
        (os.path.join('share', package_name,'param'), glob(os.path.join('rviz','*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leo',
    maintainer_email='10360882+huo-haijie@user.noreply.gitee.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_start = leo_grasp.grasp_start:main',
            'rgb_detect = leo_grasp.rgb_detect:main',
            'ros_roi_hsv = leo_grasp.ros_roi_hsv:main',
            'hsv_motion = leo_grasp.hsv_motion:main',
        ],
    },
)
