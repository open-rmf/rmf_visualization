from setuptools import setup

package_name = 'fleet_state_visualizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=['fleet_state_visualizer'],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    author='Morgan Quigley',
    author_email='morgan@osrfoundation.org',
    zip_safe=True,
    maintainer='Morgan Quigley',
    maintainer_email='morgan@osrfoundation.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='fleet_state_visualizer',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    scripts=[],
    entry_points={
        'console_scripts': [
            'fleet_state_visualizer = fleet_state_visualizer.fleet_state_visualizer:main',
        ],
    },
)
