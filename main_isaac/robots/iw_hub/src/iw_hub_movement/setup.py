from setuptools import find_packages, setup

package_name = 'iw_hub_movement'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/iw_hub.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ash-Ahn-Mechanic',
    maintainer_email='domcove9@gmail.com',
    description='IW Hub 이동 제어 패키지 (Rokey_isaac-sim 통합)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'move_to_point = iw_hub_movement.move_to_point:main',
            'axis_nav      = iw_hub_movement.axis_nav:main',
            'manual_console = iw_hub_movement.manual_console:main',
        ],
    },
)
