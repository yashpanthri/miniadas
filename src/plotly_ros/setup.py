from setuptools import find_packages, setup
#ADD
import os
from glob import glob
#END ADD

package_name = 'plotly_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

#ADD
        (os.path.join('share',package_name,
        'launch'), glob(os.path.join('launch',
        '*launch.[pxy][yma]*')))

#END ADD
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yashpanthri-unbuntu22',
    maintainer_email='yash.panthri05@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_publisher = plotly_ros.trajectory_publisher:main',
            'final_pid = plotly_ros.final_pid:main',
            'combined_plotly_dashboard = plotly_ros.combined_plotly_dashboard:main',
        ],
    },
)