from setuptools import setup

package_name = 'lrs_loitering_sync'

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
    maintainer='abarcis',
    maintainer_email='agata.barcis@tii.ae',
    description='Synchronization of loitering drones',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'loitering_sync = lrs_loitering_sync.loitering_sync:main',
            'loitering_sync_metrics = lrs_loitering_sync.loitering_sync_metrics:main',
            'go_command = lrs_loitering_sync.go_command_generator:main',
            'phase_offset = lrs_loitering_sync.phase_offset_publisher:main',
            'basestation = lrs_loitering_sync.basestation:main'
        ],
    },
)
