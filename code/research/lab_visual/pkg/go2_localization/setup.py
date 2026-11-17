from setuptools import setup
from glob import glob

package_name = 'go2_localization'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ilovehan9283',
    maintainer_email='atomyhonor@gmail.com',
    description='고정 사전 지도 기반 위치추정 (mapping 하지 않는다)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'icp_localization = go2_localization.icp_localization:main',
            'odometry_map_adapter = go2_localization.odometry_map_adapter:main',
            'odometry_monitor = go2_localization.odometry_monitor:main',
            'map_publisher = go2_localization.map_publisher:main',
            'traffic_light_detector = go2_localization.traffic_light_detector:main',
        ],
    },
)
