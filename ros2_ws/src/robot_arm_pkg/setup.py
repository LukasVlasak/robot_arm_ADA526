from setuptools import setup

package_name = 'robot_arm_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rocotics',
    maintainer_email='lukin.vlasak@seznam.cz',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_processor_node = robot_arm_pkg.img_processor_node:main',
            'arrow_movement_testing_pub = robot_arm_pkg.arrow_movement_testing_pub:main',
            'dxl_motors_controllers = robot_arm_pkg.dxl_motors_controllers:main',
            'operator = robot_arm_pkg.operator_node:main'
        ],
    },
)
