from setuptools import find_packages, setup

package_name = 'hospital_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/hardware.launch.py']),
        ('share/' + package_name + '/launch', ['launch/visualize.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/robot.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vikash',
    maintainer_email='vikash@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_control = hospital_hardware.motor_control:main',
            'motor_control_with_encoder = hospital_hardware.motor_control_with_encoder:main'
            
        ],
    },
)
