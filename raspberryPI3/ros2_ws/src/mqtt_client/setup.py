from setuptools import find_packages, setup

package_name = 'mqtt_client'

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
    maintainer='pi',
    maintainer_email='ronan.bonnet42@wanadoo.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt_client = mqtt_client.mqtt_client_node:main',
            'odom_yaw_flip = mqtt_client.odom_yaw_flip_node:main',
            'generatepath = mqtt_client.generatepath_node:main',
            'calibration_orientation = mqtt_client.calibration_orientation_node:main',
            'imu_calibration = mqtt_client.imu_calibration_publisher_node:main',
        ],
    },
)
