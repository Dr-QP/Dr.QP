from setuptools import find_packages, setup

package_name = 'drqp_robot_mcp'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'mcp[cli]>=1.26.0',
    ],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Anton Matosov',
    maintainer_email='anton.matosov@gmail.com',
    description='MCP server package for interacting with the Dr.QP simulation.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'drqp_robot_mcp = drqp_robot_mcp.server:main',
        ],
    },
)
