from setuptools import setup

package_name = 'rpi5_robot_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
      ('share/' + package_name, ['package.xml']),
      ('share/' + package_name + '/config', ['config/ekf.yaml']),
      ('share/' + package_name + '/launch', ['launch/rpi5_localization_launch.py']),
      ('share/' + package_name + '/launch', ['launch/rpi5_cant_move_sim_launch.py']),
    ],
    install_requires=[
        'setuptools',
        'gpiod',                
        'pyserial'               
    ],
    zip_safe=True,
    maintainer='lego_Dai',
    maintainer_email='gdteyuj123@gmail.com',
    description='RPi5 localization + motor control',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'rpi5_encoder_node          = rpi5_robot_localization.rpi5_encoder_node:main',
            'rpi5_imu_node              = rpi5_robot_localization.rpi5_imu_node:main',
            'rpi5_serial_bridge_node    = rpi5_robot_localization.rpi5_serial_bridge_node:main',
            'rpi5_odometry_node         = rpi5_robot_localization.rpi5_odometry_node:main',
            'rpi5_control_node          = rpi5_robot_localization.rpi5_control_node:main',
        ],
    },
)
