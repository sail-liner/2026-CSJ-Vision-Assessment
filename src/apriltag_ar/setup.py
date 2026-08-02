from glob import glob
from setuptools import find_packages, setup

package_name = 'apriltag_ar'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        (
            'share/' + package_name + '/config',
            glob('config/*.yaml') + glob('config/*.rviz')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sail',
    maintainer_email='281960055+sail-liner@users.noreply.github.com',
    description=(
        'ROS 2 AprilTag detection, pose estimation, TF, RViz and AR '
        'visualization package'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_publisher = apriltag_ar.camera_publisher:main',
            'apriltag_detector = apriltag_ar.apriltag_detector:main',
            'apriltag_direct = apriltag_ar.apriltag_direct:main',
        ],
    },
)
