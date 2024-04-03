from setuptools import setup
import os
from glob import glob
from urllib.request import urlretrieve
from setuptools import find_packages

package_name = 'yolov5_humble_fls'

setup(
    name=package_name,
    version='0.0.0',
    #packages=[package_name],
    packages= find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author="cabinx",
    maintainer='cabinx',
    maintainer_email='kaibng@mail.ustc.edu.cn',
    description='YOLOV5 object detaction with multibeam forward looking sonar on ros2 humble',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov5_ros2 = '+package_name+'.yolov5_sonar:main',
            #'yolov5_ros2_cabin = yolov5_ros2_cabin.yolov5:main',
        ],
    },
)
