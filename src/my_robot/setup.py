import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # نسخ ملفات الـ launch تلقائياً:
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # نسخ ملفات الـ urdf و xacro:
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        # نسخ ملفات الـ config والـ bridge:
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='salma',
    maintainer_email='salma@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_picker = my_robot.yolo_picker_node:main',
        ],
    },
)