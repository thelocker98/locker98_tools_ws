from setuptools import find_packages, setup

package_name = 'locker98_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='locker98',
    maintainer_email='coolemail45@proton.me',
    description='This is a collection of tools that I find useful on a regular basis.',
    license='GNU 3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "joy_teleop = locker98_tools.joy_teleop:main"
        ],
    },
)
