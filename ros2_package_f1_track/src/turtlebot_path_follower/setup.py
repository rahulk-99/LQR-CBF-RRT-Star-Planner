from setuptools import setup
import os

package_name = 'turtlebot_path_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/data', [os.path.join('data', 'final_path.txt')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kunj',
    maintainer_email='kgolwala@umd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot_path_follower = turtlebot_path_follower.turtlebot3_path_follower:main',
            'track_follower = turtlebot_path_follower.track_follower:main',
        ],
    },
)
