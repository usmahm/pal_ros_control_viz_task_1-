from setuptools import find_packages, setup

package_name = 'sensor_controller_py'

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
    maintainer='Ahmad Usman',
    maintainer_email='ajusman@student.oauife.edu.ng',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_controller_client = sensor_controller_py.sensor_controller_client:main',
            'sensor_controller_server = sensor_controller_py.sensor_controller_server:main',
            'data_publisher = sensor_controller_py.data_publisher:main',
        ],
    },
)
