from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'leo_yolov8'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name,'launch'), glob(os.path.join('launch','*launch.py'))),
	
	(os.path.join('share', package_name,'scripts'), glob(os.path.join('scripts','*.sh'))),
	(os.path.join('lib', package_name), glob(os.path.join('scripts','*.sh'))),
	
    	(os.path.join('share', package_name,'config'), glob(os.path.join('config','*.pt'))),
    	(os.path.join('lib', package_name,'config'), glob(os.path.join('config','*.pt'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nxrobo',
    maintainer_email='10360882+huo-haijie@user.noreply.gitee.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'camera_predict = leo_yolov8.camera_predict:main',
             'camera_pose = leo_yolov8.camera_pose:main',
             'camera_predict_tf = leo_yolov8.camera_predict_tf:main',
             'id_service = leo_yolov8.id_service:main',
             'camera_object = leo_yolov8.camera_object:main',

        ],
    },
)
