from setuptools import find_packages, setup

package_name = 'drqp_brain'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],  # must be here, otherwise colcon will skip tests
    zip_safe=True,
    maintainer='Anton Matosov',
    maintainer_email='anton.matosov@gmail.com',
    description='IK solvers and other high level control algorithms',
    license='MIT',
)
