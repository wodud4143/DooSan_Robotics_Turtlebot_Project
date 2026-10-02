from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'cctv_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LEE JAE YOUNG',
    maintainer_email='wodud4143@naver.com',
    description='CCTV 쓰러짐/혼잡/인원수 감지 노드',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cctv_detector_node = cctv_perception.cctv_detector_node:main',
        ],
    },
)
