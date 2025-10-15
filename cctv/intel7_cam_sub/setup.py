from setuptools import setup

package_name = 'intel7_cam_sub'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='you@example.com',
    description='Subscriber for camera detections JSON',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'det_subscriber = intel7_cam_sub.subscriber_node:main',
        ],
    },
)
