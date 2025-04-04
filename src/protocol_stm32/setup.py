from setuptools import find_packages, setup

package_name = 'protocol_stm32'

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
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		"telemetry_stm32 = protocol_stm32.telemetry_stm32:main",
		"heartbeat = protocol_stm32.heartbeat:main",
		"driver = protocol_stm32.driver:main",
		"driver_stm32 = protocol_stm32.driver_stm32:main"
        ],
    },
)
