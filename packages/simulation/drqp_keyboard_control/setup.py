from setuptools import find_packages, setup

package_name = 'drqp_keyboard_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'pygame>=2.5',
    ],
    extras_require={
        'test': ['pytest'],
    },
    zip_safe=True,
    maintainer='Anton Matosov',
    maintainer_email='anton.matosov@gmail.com',
    description='GUI keyboard and virtual stick controller for Dr.QP simulation.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'drqp_keyboard_control = drqp_keyboard_control.keyboard_control_node:main',
        ],
    },
)
