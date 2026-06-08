from setuptools import find_packages, setup

package_name = 'odom_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kreacher',
    maintainer_email='shourybanner@gmail.com',
    description='TODO: Package description',
    license='Apache2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        #'apfa_final = odom_publisher.apf_depthcam:main',
        'apfa_1 = odom_publisher.odom_subscriber:main',
        'apfa_2 = odom_publisher.odom_subscriber2:main',
        'apfa_3 = odom_publisher.odom_subscriber3:main',
        'apfa_final = odom_publisher.odom_subscriber4:main',
        'apfa_depth = odom_publisher.odom_subscriver_depthcam2:main',
        'apfa_depth1 = odom_publisher.odom_subscriber_depth_cam:main',
        'apfa_depth2 = odom_publisher.odom_subscriber_depth_cam_copy:main',
        'apfa_depth_final = odom_publisher.apfa_depth_final:main',
        'apfa_depth_final2 = odom_publisher.apfa_depth_final2:main',

        ],
    },
)
