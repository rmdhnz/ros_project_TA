from setuptools import find_packages, setup

package_name = 'my_robot_controller'

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
    maintainer='divspan',
    maintainer_email='ramadhanfadhly2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtle_controller = my_robot_controller.turtle_controller:main",
            "imu_node = my_robot_controller.imu_sens:main",
            "lidar_node = my_robot_controller.lidar_node:main",
            "odom_node = my_robot_controller.odom_node:main",
            "fusion_node = my_robot_controller.fusion_node:main",
            "ekf_fusion_node = my_robot_controller.ekf_fusion:main",
            "path_listener = my_robot_controller.path_listener:main",
            "nav_test_node = my_robot_controller.nav_tes:main",
            "path_saver =my_robot_controller.save_path:main",
            "imu_filtered_node=my_robot_controller.imu_filtered:main",
            "odom_filtered_node=my_robot_controller.odom_filtered:main",
            "imu_raw_data=my_robot_controller.imu_raw_data:main",
            "odom_raw_data=my_robot_controller.odom_raw_data:main",
            "obstacle_monitor=my_robot_controller.obstacle_monitor:main"
        ],
    },
)
