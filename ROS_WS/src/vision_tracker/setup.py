from setuptools import setup

package_name = 'vision_tracker'

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
    maintainer='micha',
    maintainer_email='michaelasteinklw@t-online.de',
    description='Vision tracker',
    license='MIT',
    entry_points={
        'console_scripts': [
            'hotspot_tracker = vision_tracker.hotspot_tracker:main'
        ],
    },
)
