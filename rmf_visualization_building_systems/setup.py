from setuptools import setup

package_name = 'rmf_visualization_building_systems'

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
    maintainer='yadu',
    maintainer_email='yadunund@openrobotics.org',
    description='A visualizer for doors and lifts',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rmf_visualization_building_systems = \
                rmf_visualization_building_systems.rmf_visualization_building_systems:main'
        ],
    },
)
