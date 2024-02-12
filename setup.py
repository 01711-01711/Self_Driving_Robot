from setuptools import setup

package_name = 'mech0020_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mateusz Zawila',
    maintainer_email='mateuszzawila01@gmail.com',
    description='ROS 2 package for MECH0020 Robot',
    license='License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = mech0020_robot.publisher:main',
            'subscriber = mech0020_robot.subscriber:main',
        ],
    },
)
