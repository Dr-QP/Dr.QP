from setuptools import find_packages, setup

package_name = 'drqp_brain'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anton Matosov',
    maintainer_email='anton.matosov@gmail.com',
    description='IK solvers and other high level control algorithms',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'one_leg_ik = drqp_brain.one_leg_ik:main',
        ],
    },
)
